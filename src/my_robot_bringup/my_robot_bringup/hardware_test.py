"""Get init position from launch configurations
"""


from controller_manager_msgs.srv import SwitchController, ListControllers
from rclpy.executors import ExternalShutdownException
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from irobot_create_msgs.action import Undock, Dock
from ros_gz_interfaces.msg import Contacts
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import ClearEntireCostmap
import time
import datetime
from enum import Enum
import math
import subprocess
import csv
import os
from pathlib import Path
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


class SequenceState(Enum):
    IDLE = 0
    UNDOCKING = 1
    UNDOCKING_IN_PROGRESS = 2
    WAITING_AFTER_UNDOCK = 3
    INITIALIZING_POSE = 4
    CREATE_PATH = 5
    NAVIGATING = 6
    COMPLETED = 7
    FAILED = 8
    RESTART = 9


class BarnOneShot(Node):
    def __init__(self):
        super().__init__('Barn_one_shot')

        # Use reentrant callback group for concurrent action calls

        # Keep track of results:
        self.trial_results = []
        self._trial_result = None
        self.callback_group = ReentrantCallbackGroup()

        # State management
        self.current_state = SequenceState.IDLE

        self._nav_timeout_counter = 0
        self._nav_timeout_limit = 5

        self._nav_feedback_counter = 0
        self._nav_feedback_limit = 30
        self.prev_distance_flag = False

        self.total_lg = 0
        self.current_lg_counter = 0
        self.current_lg_xy = (0, 0)

        # Action clients
        self.undock_client = ActionClient(
            self, Undock, '/undock',
            callback_group=self.callback_group
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        # publishs converted barn map to nav stack
        self.path_publisher = self.create_publisher(Path, '/plan_barn', 10)
        self.path_og_publisher = self.create_publisher(Path, '/plan_barn_og', 10)

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.bumper_sub = self.create_subscription(
            Contacts, '/bumper_contact', self.bumper_callback,
            10)

        self.feedback_sub = self.create_subscription(NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback',
                                                     self.nav_feedback_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Current position in map frame
        self.current_map_x = 0.0
        self.current_map_y = 0.0

        self.final_goal_x = None
        self.final_goal_y = None

        self.goal_tolerance_xy = 1

        self.amcl_pose_received = False
        self.pose_stable_count = 0
        self._nav_goal_handle = None  # Track navigation goal handle for cancellationrestart
        self._restart_in_progress = False  # Prevent restart loops

        self.config_radius = None
        self.config_num_valid_obstacles = None
        self.config_offset = None
        self.config_record_csv = None
        self.config_model_path = None
        # gazebo path
        self.gazebo_path = None
        self.gazebo_path_og = None
        # Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('wait_after_undock', 2.0)
        self.declare_parameter('pose_init_delay', 1.0)

        self.collision_detected = False
        self.prev_distance = 0
        self.distance_remaining = 0
        self.goal_x = 0
        self.goal_y = 0


        self.bag_proc = None
        self.bag_outdir = None
        self.bag_topics = ["/odom", "/cmd_vel", "/plan_barn", "/plan_barn_odom", "/tf", "/tf_static", "/scan"]
        # Timer for state machine
        self.state_timer = self.create_timer(
            1.0, self.state_machine_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Hardware Node initialized')

        # Start the sequence
        self.start_sequence()

    def start_sequence(self):
        """Start the navigation sequence"""
        self.get_logger().info('Starting navigation sequence...')
        self.current_state = SequenceState.INITIALIZING_POSE  # Go directly to UNDOCKING
        self.retry_count = 0

    def state_machine_callback(self):
        """Main state machine callback with debug logging"""
        self.get_logger().info(
            f"State machine tick: current_state = {self.current_state}")

        if self.current_state == SequenceState.IDLE:
            # IDLE state should transition to UNDOCKING
            self.get_logger().info("In IDLE state, transitioning to UNDOCKING")
            self.current_state = SequenceState.UNDOCKING
        elif self.current_state == SequenceState.UNDOCKING:
            self.handle_undocking()
        elif self.current_state == SequenceState.UNDOCKING_IN_PROGRESS:
            self.handle_undocking_in_progress()
        elif self.current_state == SequenceState.WAITING_AFTER_UNDOCK:
            self.get_logger().info("Undocking sequence completed successfully!")
            self.current_state = SequenceState.INITIALIZING_POSE
        elif self.current_state == SequenceState.INITIALIZING_POSE:

            self.bag_proc = None
            self.bag_outdir = None
            self.bag_topics = ["/odom", "/cmd_vel", "/plan_barn", "/plan_barn_odom", "/tf", "/tf_static", "/scan"]
            self.handle_pose_initialization()
        elif self.current_state == SequenceState.NAVIGATING:
            self.handle_navigation()
        elif self.current_state == SequenceState.COMPLETED:
            if not hasattr(self, '_completed_logged'):
                self.get_logger().info("Node completed - staying alive")
                self._completed_logged = True
                self._failed_logged = True
                self._failed_logged = True
                self.record_results()
                self.stop_bag() 
                self.terminate()
                self.current_state = SequenceState.RESTART
        elif self.current_state == SequenceState.FAILED:
            if not hasattr(self, '_failed_logged'):
                self.get_logger().info("Node failed - staying alive")
                self._failed_logged = True
                self.record_results()      
                self.stop_bag() 
                self.terminate()
                self.current_state = SequenceState.RESTART
        elif self.current_state == SequenceState.RESTART:
            self.restart_trial()

    def handle_undocking(self):
        """Handle undocking phase - only sends command once"""
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Undock action server not available')
            return

        self.get_logger().info('Sending undock command...')
        goal_msg = Undock.Goal()
        self._undock_future = self.undock_client.send_goal_async(goal_msg)
        self._undock_future.add_done_callback(
            self.undock_goal_response_callback)

        self.current_state = SequenceState.UNDOCKING_IN_PROGRESS
        self._undock_start_time = time.time()

    def handle_undocking_in_progress(self):
        """Handle timeout while undocking is in progress"""
        if time.time() - self._undock_start_time > 60.0:
            self.get_logger().warn('Undock action timed out')

    def undock_goal_response_callback(self, future):
        """Callback for undock goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected')
            return

        self.get_logger().info('Undock goal accepted, waiting for completion...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        """Callback for undock result"""
        result = future.result().result
        self.get_logger().info(f'Undock completed with result: {result}')

        self.current_state = SequenceState.WAITING_AFTER_UNDOCK
        self._wait_start_time = time.time()

    def handle_pose_initialization(self):
        """Handle initial pose setting with retry mechanism"""

        # Initialize retry variables if not already done
        if not hasattr(self, '_pose_retry_count'):
            self._pose_retry_count = 0
            self._max_retries = 5
            self._retry_interval = 5.0  # seconds between retries
            self._pose_sent = False
            self._pose_init_time = None
            # Reset AMCL flags when starting pose initialization
            self.amcl_pose_received = False
            self.pose_stable_count = 0

        # Check if we should send/retry the pose
        should_send_pose = (
            not self._pose_sent or  # First attempt
            (self._pose_sent and
             self._pose_init_time and
             time.time() - self._pose_init_time >= self._retry_interval and
             not self.amcl_pose_received and
             self._pose_retry_count < self._max_retries)
        )

        if should_send_pose:
            self._pose_retry_count += 1

            self.get_logger().info(
                f'Setting initial pose (attempt {self._pose_retry_count}/{self._max_retries})...')

            # Reset AMCL flags for this attempt
            self.amcl_pose_received = False
            self.pose_stable_count = 0

            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()

            # Set position
            initial_pose.pose.pose.position.x = self.get_parameter(
                'initial_x').value
            initial_pose.pose.pose.position.y = self.get_parameter(
                'initial_y').value
            initial_pose.pose.pose.position.z = 0.0

            # Set orientation (convert yaw to quaternion)
            yaw = self.get_parameter('initial_yaw').value
            initial_pose.pose.pose.orientation.x = 0.0
            initial_pose.pose.pose.orientation.y = 0.0
            initial_pose.pose.pose.orientation.z = math.sin(yaw/2)
            initial_pose.pose.pose.orientation.w = math.cos(yaw/2)

            # Set covariance (slightly increase for retries to help AMCL accept)
            covariance_multiplier = 1.0 + (self._pose_retry_count - 1) * 0.5
            initial_pose.pose.covariance = [0.0] * 36
            initial_pose.pose.covariance[0] = 0.25 * \
                covariance_multiplier   # x
            initial_pose.pose.covariance[7] = 0.25 * \
                covariance_multiplier   # y
            initial_pose.pose.covariance[35] = 0.068 * \
                covariance_multiplier  # yaw

            self.initial_pose_pub.publish(initial_pose)
            self._pose_sent = True
            self._pose_init_time = time.time()

        elif self._pose_sent and self._pose_init_time:
            # Monitor the current attempt
            delay = 13.0  # Wait time after sending pose
            elapsed = time.time() - self._pose_init_time

            self.get_logger().info(
                f'Waiting for AMCL (attempt {self._pose_retry_count}/{self._max_retries}): '
                f'received={self.amcl_pose_received}, elapsed={elapsed:.1f}s/{delay}s'
            )

            # Check for success
            if self.amcl_pose_received and elapsed >= delay:
                self.get_logger().info(
                    f'AMCL pose confirmed after {self._pose_retry_count} attempts, proceeding to navigation...')
                self.current_state = SequenceState.NAVIGATING
                # Reset retry variables for next time
                self._reset_pose_retry_vars()

            # Check for timeout on current attempt (triggers retry if retries left)
            elif elapsed >= 30.0:
                if self._pose_retry_count >= self._max_retries:
                    self.get_logger().error(
                        f'AMCL pose initialization failed after {self._max_retries} attempts')
                    self._trial_result = "AMCL TIMEOUT - MAX RETRIES EXCEEDED"
                    self.current_state = SequenceState.FAILED
                    self._reset_pose_retry_vars()
                else:
                    self.get_logger().warn(
                        f'Attempt {self._pose_retry_count} timed out, will retry...')
                    # Reset for next attempt (will trigger retry on next call)
                    self._pose_sent = False
                    self._pose_init_time = None

    def _reset_pose_retry_vars(self):
        """Reset all pose retry related variables"""
        self._pose_retry_count = 0
        self._pose_sent = False
        self._pose_init_time = None

    def handle_navigation(self):
        """Handle navigation to goal pose"""
        if not hasattr(self, '_nav_sent') or not self._nav_sent:
            if not self.navigate_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation action server not available')
                self._nav_timeout_counter += 1

                if self._nav_timeout_counter >= self._nav_timeout_limit:
                    self.get_logger().error('Nav server has timed out')
                    self._trial_result = "NAV_SERVER_UNAVAILABLE"
                    self.current_state = SequenceState.FAILED
                return

            self.get_logger().info('Sending navigation goal...')

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            goal_x = self.get_parameter('goal_x').value
            goal_y = self.get_parameter('goal_y').value
            goal_yaw = self.get_parameter('goal_yaw').value
            # Set goal position
            goal_msg.pose.pose.position.x = goal_x
            goal_msg.pose.pose.position.y = goal_y
            goal_msg.pose.pose.position.z = 0.0

            # Set goal orientation
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = math.sin(goal_yaw/2)
            goal_msg.pose.pose.orientation.w = math.cos(goal_yaw/2)

            self.goal_x = goal_msg.pose.pose.position.x
            self.goal_y = goal_msg.pose.pose.position.y

            if not self.is_cmd_vel_subscribed():
                self.get_logger().warn("cmd_vel has no subscribers — controller likely not active yet")
                return  # or transition to FAILED if it's unrecoverable

            self._nav_future = self.navigate_client.send_goal_async(goal_msg)
            self._nav_future.add_done_callback(self.nav_goal_response_callback)
            self._nav_sent = True
            self._nav_start_time = time.time()

        elif time.time() - self._nav_start_time > 300.0:
            self.get_logger().warn('Navigation action timed out')
            if self._nav_goal_handle:
                self._nav_goal_handle.cancel_goal_async()
            self._trial_result = "TIMEOUT"
            self.current_state = SequenceState.FAILED
            self._nav_sent = False
        print("have not timed out _______________________")
        self.final_goal_tracker()
        if self.collision_detected:
            self.trial_result = COLLISION
            self.current_state = SequeneceState.FAILED
        # method get_feeedback from navigate client, distance remaining

    def nav_goal_response_callback(self, future):
        """Callback for navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self._nav_goal_handle = None
            return

        self.get_logger().info('Navigation goal accepted')
        self._nav_goal_handle = goal_handle  # Store for potential cancellation
        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Callback for navigation result"""
        self.get_logger().info("nav_result_callback called!")

        try:
            if future.cancelled():
                self.get_logger().warn("Navigation goal was cancelled before completion")
                self._trial_result = "CANCELLED"
                self.current_state = SequenceState.RESTART
                self._nav_sent = False
                return

            if not future.done():
                self.get_logger().warn("Navigation result future is not done yet")
                return

            result_msg = future.result()
            if result_msg is None:
                self.get_logger().error("Navigation result returned None")
                self._trial_result = "ERROR"
                self.current_state = SequenceState.RESTART
                self._nav_sent = False
                return

            status = result_msg.status
            result = result_msg.result

            self.get_logger().info(f"Nav status: {status}")
            if status == 4:  # SUCCESS
                self.get_logger().info(f'Navigation success: {result}')
                self._trial_result = "SUCCESS"

            elif status == 5:  # ABORTED - This is the key detection for planning failures and other aborts
                self.get_logger().error(f'Navigation aborted: This often means no valid path could be found by the planner or the controller failed to follow.')
                # Check collision_detected if it's an aborted case due to bumper
                if self.collision_detected:
                    self._trial_result = "COLLISIONS"
                    self.get_logger().error("Navigation aborted due to collision!")
                else:
                    # More specific for planning/controller issues
                    self._trial_result = "NAV_ABORTED_PLANNING_FAIL"
                    self.get_logger().error(
                        "Navigation aborted, likely planning or controller failure (no collision detected).")
            # CANCELED - This happens if you explicitly cancel it, e.g., due to timeout.
            elif status == 6:
                self.get_logger().warn(f"Navigation was cancelled for trial")
                # If _trial_result was already set to "TIMEOUT", keep it. Otherwise, set to CANCELLED.
                if self._trial_result != "TIMEOUT":
                    self._trial_result = "FAILURE"
            else:
                self.get_logger().error(
                    f'Navigation failed for trial  with unknown status: {status}.')
                self._trial_result = "NAV_UNKNOWN_FAILURE"

            # This does ot accout for collisions
            self.get_logger().info(f"trial_result : {self._trial_result}")
            self.current_state = SequenceState.COMPLETED
            self._nav_sent = False

        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self._trial_result = "ERROR"
            self.current_state = SequenceState.FAILED
            self._nav_sent = False

    def amcl_pose_callback(self, msg):
        """Monitor AMCL pose for stability"""
        self.amcl_pose_received = True
        self.pose_stable_count += 1

    def bumper_callback(self, msg):
        """Monitor for bumper collisions"""
        if msg.contacts and self.current_state == SequenceState.NAVIGATING:  # collision detected
            self.get_logger().warn(f"Collision! {len(msg.contacts)} contact")

            for i, contact in enumerate(msg.contacts):
                self.get_logger().info(
                    f"Collision {i} {contact.collision1.name} hit {contact.collision2.name}")
                self.get_logger().info(
                    f"This collision occured at {contact.positions}")
            self.collision_detected = True
            self._trial_result = "COLLISION"
            self.current_state = SequenceState.FAILED

    def is_cmd_vel_subscribed(self):
        """Check if any node is subscribed to /cmd_vel."""
        info = self.get_publishers_info_by_topic('/cmd_vel')
        return len(info) > 0

    def yaml_reader(self):
        """Read the configs from config.yaml"""
        filepath = os.path.join(os.path.expanduser(
            '~'), 'ros_ws', 'config.yaml')  # Fixed typo: trail -> trial
        with open(filepath, "r") as file:
            config = yaml.safe_load(file)

            # Example access
            self.config_radius = config["RADIUS"]
            self.config_num_valid_obstacles = config["NUM_VALID_OBSTACLES"]
            self.config_offset = config["OFFSET"]
            self.config_record_csv = config["CSV_FILE"]
            self.config_model_path = config["MODEL_PATH"]

            print(
                f"Loaded: RADIUS={self.config_radius}, NUM={self.config_num_valid_obstacles}, OFFSET={self.config_offset}")


    def nav_feedback_callback(self, msg):
        """This function checks if the robot is making progress towards the goal pose, terminates if not"""
        if not self.prev_distance_flag:

            initial_x = self.get_parameter('initial_x').value
            initial_y = self.get_parameter('initial_y').value
            goal_x = self.goal_x
            goal_y = self.goal_y

            self.get_logger().info(f"Initial pose: ({initial_x}, {initial_y})")
            self.get_logger().info(f"Goal pose: ({goal_x}, {goal_y})")

            self.prev_distance = math.sqrt(
                (goal_x - initial_x)**2 + (goal_y - initial_y)**2)
            self.get_logger().info(
                f"Calculated initial distance: {self.prev_distance}")
            self.prev_distance_flag = True
            self.last_progress_check_time = time.time()
            self.progress_check_interval = 1.0  # Check every 1 second
            return

        self.distance_remaining = self.local_goal_tracker()
        self.final_goal_tracker()
        current_time = time.time()

        if current_time - self.last_progress_check_time >= self.progress_check_interval:

            self.get_logger().info(
                f'Dist to goal {self.distance_remaining} and prev distance {self.prev_distance}')
            if self.distance_remaining > self.prev_distance:
                self._nav_feedback_counter += 1
                self.get_logger().info(
                    f'Increasing counter {self._nav_feedback_counter}')
                self.prev_distance = self.distance_remaining
                if self._nav_feedback_counter >= self._nav_feedback_limit:
                    self.get_logger().info(
                        f'Have not made progress within {self._nav_feedback_limit} tries, failure')
                    self._trial_result = "FAILURE"
                    self.current_state = SequenceState.FAILED
            else:
                self.prev_distance = self.distance_remaining
                self._nav_feedback_counter = 0

            self.last_progress_check_time = current_time


    def final_goal_tracker(self):
        if self.final_goal_x == None and self.gazebo_path:
            self.final_goal_x = self.gazebo_path.poses[-1].pose.position.x
            self.final_goal_y = self.gazebo_path.poses[-1].pose.position.y

        dx = self.current_map_x - self.final_goal_x
        dy = self.current_map_y - self.final_goal_y
        final_distance = ((dx*dx) + (dy*dy)**.5)
        print("*******************************************")
        print(f"final distance {final_distance}")
        if final_distance < self.goal_tolerance_xy:
            self._trial_result = "SUCCESS"
            self.current_state = SequenceState.COMPLETED


    def odom_callback(self, odom_msg):

        if self.current_state != SequenceState.NAVIGATING:
            return
        try:
            # Create PoseStamped from odometry message
            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose

            # Transform to map frame
            map_pose = self.tf_buffer.transform(pose_stamped, 'map')

            # Update current map coordinates
            self.current_map_x = map_pose.pose.position.x
            self.current_map_y = map_pose.pose.position.y

            self.get_logger().info(
                f'Map position: x={self.current_map_x:.2f}, y={self.current_map_y:.2f}')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'Could not transform odom to map: {str(e)}')


    def start_bag(self, outdir: str):
            if self.bag_proc and self.bag_proc.poll() is None:
                self.get_logger().warn("rosbag2 already running; not starting another.")
                return
            try:
                # Make sure only the *parent* exists
                parent = os.path.dirname(outdir) or "."
                os.makedirs(parent, exist_ok=True)

                # (optional) capture a log so you see errors instead of /dev/null
                log_path = os.path.join(parent, f"{os.path.basename(outdir)}.record.log")
                self.get_logger().info(f"Starting rosbag2 → {outdir} (log: {log_path})")
                logf = open(log_path, "wb", buffering=0)

                self.bag_outdir = outdir
                self.bag_proc = subprocess.Popen(
                    self._bag_cmd(outdir),          # e.g. ["ros2","bag","record","-o",outdir,...]
                    preexec_fn=os.setsid,
                    stdout=logf,
                    stderr=subprocess.STDOUT,
                )

                # quick health check
                time.sleep(1.0)
                if self.bag_proc.poll() is not None:
                    self.get_logger().error(
                        f"rosbag2 exited with code {self.bag_proc.returncode}. See log: {log_path}"
                    )
                    self.bag_proc = None
            except Exception as e:
                self.get_logger().error(f"Failed to start rosbag2: {e}")
                self.bag_proc = None

    def stop_bag(self, grace_sec: float = 10.0):
        proc = self.bag_proc
        if not proc:
            return
        if proc.poll() is not None:  # already exited
            self.bag_proc = None
            return

        try:
            pgid = os.getpgid(proc.pid)
            self.get_logger().info("Stopping rosbag2 (SIGINT)...")
            os.killpg(pgid, signal.SIGINT)  # graceful stop → writes metadata.yaml
            try:
                proc.wait(timeout=grace_sec)
                self.get_logger().info("rosbag2 stopped cleanly.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("rosbag2 did not stop in time; escalating (SIGTERM → SIGKILL if needed).")
                os.killpg(pgid, signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error while stopping rosbag2: {e}")
        finally:
            self.bag_proc = None

def main():
    rclpy.init()

    try:
        node = BarnOneShot()

        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Received KeyboardInterrupt, shutting down gracefully...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Clean shutdown
        try:
            if rclpy.ok():  # Check if ROS2 context is still valid
                node.destroy_node()  # Clean up your node
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")


if __name__ == '__main__':
    main()
