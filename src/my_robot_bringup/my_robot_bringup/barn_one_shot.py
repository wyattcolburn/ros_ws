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
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import ClearEntireCostmap
import time,datetime
from enum import Enum
import math
import subprocess
import csv
import os
from pathlib import Path
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
class SequenceState(Enum):
    IDLE = 0
    UNDOCKING = 1
    UNDOCKING_IN_PROGRESS = 2
    WAITING_AFTER_UNDOCK = 3
    INITIALIZING_POSE = 4
    CREATE_PATH =5
    NAVIGATING = 6
    COMPLETED = 7
    FAILED = 8

class BarnOneShot(Node):
    def __init__(self):
        super().__init__('One_Shot')
        
        # Use reentrant callback group for concurrent action calls

        # Keep track of results:
        self.trial_results = []
        self._trial_result = None
        self.callback_group = ReentrantCallbackGroup()
        
        # State management
        self.current_state = SequenceState.IDLE
        self.num_trials = 10 
        self.current_trial = 0

        self._nav_timeout_counter = 0
        self._nav_timeout_limit = 5

        self._nav_feedback_counter = 0
        self._nav_feedback_limit = 10
        self.prev_distance_flag = False
            
        # Action clients
        self.undock_client = ActionClient(
            self, Undock, '/undock', 
            callback_group=self.callback_group
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        # publishs converted barn map to nav stack
        self.path_publisher = self.create_publisher(Path, '/plan', 10)

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.bumper_sub = self.create_subscription(
            Contacts, '/bumper_contact', self.bumper_callback,
                10)

        self.feedback_sub = self.create_subscription(NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback',
                                                     self.nav_feedback_callback, 10)


        self.amcl_pose_received = False
        self.pose_stable_count = 0
        self._nav_goal_handle = None  # Track navigation goal handle for cancellationrestart
        self._restart_in_progress = False  # Prevent restart loops
        
        # Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('wait_after_undock', 2.0)
        self.declare_parameter('pose_init_delay', 1.0)
        self.declare_parameter('world_num', 0)

        self.world_num = self.get_parameter('world_num').value
        self.prev_distance = 0
        self.distance_remaining = 0
        # Timer for state machine
        self.state_timer = self.create_timer(
            1.0, self.state_machine_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('ONE SHOT Sequence Node initialized')
        
        # Start the sequence
        self.start_sequence()

    def start_sequence(self):
        """Start the navigation sequence"""
        self.get_logger().info('Starting navigation sequence...')
        self.current_state = SequenceState.UNDOCKING  # Go directly to UNDOCKING
        self.retry_count = 0

    def state_machine_callback(self):
        """Main state machine callback with debug logging"""
        self.get_logger().info(f"State machine tick: current_state = {self.current_state} on trial {self.current_trial}")
        
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
            self.handle_pose_initialization()    
        elif self.current_state == SequenceState.CREATE_PATH:
            gazebo_path = self.load_barn_path(self.world_num)
            self.path_publisher.publish(gazebo_path)
        elif self.current_state == SequenceState.NAVIGATING:
            self.handle_navigation()
        elif self.current_state == SequenceState.COMPLETED:
            if not hasattr(self, '_completed_logged'):
                self.get_logger().info("Node completed - staying alive")
                self._completed_logged = True
                self._failed_logged = True
                self._failed_logged = True
                self.record_results()
                self.terminate()
        elif self.current_state == SequenceState.FAILED:
            if not hasattr(self, '_failed_logged'):
                self.get_logger().info("Node failed - staying alive")
                self._failed_logged = True
                self.record_results()
                self.terminate()
    def handle_undocking(self):
        """Handle undocking phase - only sends command once"""
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Undock action server not available')
            return
        
        self.get_logger().info('Sending undock command...')
        goal_msg = Undock.Goal()
        self._undock_future = self.undock_client.send_goal_async(goal_msg)
        self._undock_future.add_done_callback(self.undock_goal_response_callback)
        
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
        """Handle initial pose setting"""
        if not hasattr(self, '_pose_sent') or not self._pose_sent:
            # Reset AMCL flags when starting pose initialization
            self.amcl_pose_received = False
            self.pose_stable_count = 0
            
            self.get_logger().info('Setting initial pose...')
            
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set position
            initial_pose.pose.pose.position.x = self.get_parameter('initial_x').value
            initial_pose.pose.pose.position.y = self.get_parameter('initial_y').value
            initial_pose.pose.pose.position.z = 0.0
            
            # Set orientation (convert yaw to quaternion)
            yaw = self.get_parameter('initial_yaw').value
            initial_pose.pose.pose.orientation.x = 0.0
            initial_pose.pose.pose.orientation.y = 0.0
            initial_pose.pose.pose.orientation.z = math.sin(yaw/2)
            initial_pose.pose.pose.orientation.w = math.cos(yaw/2)
            
            # Set covariance
            initial_pose.pose.covariance = [0.0] * 36
            initial_pose.pose.covariance[0] = 0.25   # x
            initial_pose.pose.covariance[7] = 0.25   # y
            initial_pose.pose.covariance[35] = 0.068 # yaw
            
            self.initial_pose_pub.publish(initial_pose)
            self._pose_sent = True
            self._pose_init_time = time.time()
            
        elif hasattr(self, '_pose_init_time'):
            delay = 10.0
            elapsed = time.time() - self._pose_init_time
            
            self.get_logger().info(f'Waiting for AMCL: received={self.amcl_pose_received}, elapsed={elapsed:.1f}s/{delay}s')
           
            if (elapsed >= 30):
                # timeout not working correctly
                self._trial_result = "AMCL TIMEOUT"
                self.current_state = SequenceState.FAILED
            if self.amcl_pose_received and elapsed >= delay:
                self.get_logger().info('AMCL pose confirmed, proceeding to navigation...')
                self.current_state = SequenceState.CREATE_PATH
                self._pose_sent = False

    def handle_navigation(self):
        """Handle navigation to goal pose"""
        if not hasattr(self, '_nav_sent') or not self._nav_sent:
            if not self.navigate_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation action server not available')
                self._nav_timeout_counter+=1
                
                if self._nav_timeout_counter >= self._nav_timeout_limit:
                    self.get_logger().error('Nav server has timed out')
                    self._trial_result = "NAV_SERVER_UNAVAILABLE"
                    self.current_state = SequenceState.FAILED
                return
            
            self.get_logger().info('Sending navigation goal...')
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set goal position
            goal_msg.pose.pose.position.x = self.get_parameter('goal_x').value
            goal_msg.pose.pose.position.y = self.get_parameter('goal_y').value
            goal_msg.pose.pose.position.z = 0.0
            
            # Set goal orientation
            goal_yaw = self.get_parameter('goal_yaw').value
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = math.sin(goal_yaw/2) # to take the initial goal pose from launch file
            goal_msg.pose.pose.orientation.w = math.cos(goal_yaw/2) 
            if not self.is_cmd_vel_subscribed():
                self.get_logger().warn("cmd_vel has no subscribers — controller likely not active yet")
                return  # or transition to FAILED if it's unrecoverable


            self._nav_future = self.navigate_client.send_goal_async(goal_msg)
            self._nav_future.add_done_callback(self.nav_goal_response_callback)
            self._nav_sent = True
            self._nav_start_time = time.time()

        elif time.time() - self._nav_start_time > 120.0:
            self.get_logger().warn('Navigation action timed out')
            if self._nav_goal_handle:
                self._nav_goal_handle.cancel_goal_async()
            self._trial_result = "TIMEOUT"
            self.current_state = SequenceState.FAILED
            self._nav_sent = False
        ## method get_feeedback from navigate client, distance remaining
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

            elif status == 5: # ABORTED - This is the key detection for planning failures and other aborts
                self.get_logger().error(f'Navigation aborted for trial {self.current_trial + 1}. This often means no valid path could be found by the planner or the controller failed to follow.')
                # Check collision_detected if it's an aborted case due to bumper
                if self.collision_detected:
                    self._trial_result = "COLLISIONS"
                    self.get_logger().error("Navigation aborted due to collision!")
                else:
                    self._trial_result = "NAV_ABORTED_PLANNING_FAIL" # More specific for planning/controller issues
                    self.get_logger().error("Navigation aborted, likely planning or controller failure (no collision detected).")
            elif status == 6: # CANCELED - This happens if you explicitly cancel it, e.g., due to timeout.
                self.get_logger().warn(f"Navigation was cancelled for trial {self.current_trial + 1}.")
                # If _trial_result was already set to "TIMEOUT", keep it. Otherwise, set to CANCELLED.
                if self._trial_result != "TIMEOUT":
                    self._trial_result = "FAILURE" 
            else:
                self.get_logger().error(f'Navigation failed for trial {self.current_trial + 1} with unknown status: {status}.')
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
        if msg.contacts and self.current_state == SequenceState.NAVIGATING: # collision detected
            self.get_logger().warn(f"Collision! {len(msg.contacts)} contact")

            for i, contact in enumerate(msg.contacts):
                self.get_logger().info(f"Collision {i} {contact.collision1.name} hit {contact.collision2.name}")
                self.get_logger().info(f"This collision occured at {contact.positions}")

            self.collision_detected = True
            self.current_state = SequenceState.FAILED
    
    def is_cmd_vel_subscribed(self):
        """Check if any node is subscribed to /cmd_vel."""
        info = self.get_publishers_info_by_topic('/cmd_vel')
        return len(info) > 0
    def record_results(self):
        """Records the results of the current trial into a CSV"""
        
        # Fix 1: Properly expand the path and ensure directory exists
        filepath = Path.home() / 'ros_ws' / 'trial_results.csv'  # Fixed typo: trail -> trial
        
        # Fix 2: Create directory if it doesn't exist
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        if filepath.exists():  # Use filepath.exists() instead of os.path.exists()
            print(f"File already exists at {filepath}, appending current trial {self._trial_result}")
            with open(filepath, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    timestamp, 
                    self.get_parameter('initial_x').value,    # Fix 4: Add .value
                    self.get_parameter('initial_y').value,
                    self.get_parameter('initial_yaw').value,
                    self.get_parameter('goal_x').value,
                    self.get_parameter('goal_y').value,
                    self.get_parameter('goal_yaw').value,
                    self._trial_result
                ])
        else:
            print(f"File {filepath} does not exist, creating file and header")
            with open(filepath, 'w', newline='') as csvfile:  # Use 'w' for new file
                writer = csv.writer(csvfile)
                # Fix 3: Add missing comma
                writer.writerow(['timestamp', 'initial_x', 'initial_y', 'initial_yaw', 
                               'goal_x', 'goal_y', 'goal_yaw', 'trial_result'])
                writer.writerow([
                    timestamp,
                    self.get_parameter('initial_x').value,
                    self.get_parameter('initial_y').value,
                    self.get_parameter('initial_yaw').value,
                    self.get_parameter('goal_x').value,
                    self.get_parameter('goal_y').value,
                    self.get_parameter('goal_yaw').value,
                    self._trial_result
                ])
        return
    def nav_feedback_callback(self, msg):
        """This function checks if the robot is making progress towards the goal pose, terminates if not"""
        if not self.prev_distance_flag:

            initial_x = self.get_parameter('initial_x').value 
            initial_y = self.get_parameter('initial_y').value
            goal_x = self.get_parameter('goal_x').value
            goal_y = self.get_parameter('goal_y').value

            self.get_logger().info(f"Initial pose: ({initial_x}, {initial_y})")
            self.get_logger().info(f"Goal pose: ({goal_x}, {goal_y})")

            self.prev_distance = math.sqrt((goal_x - initial_x)**2 + (goal_y - initial_y)**2)
            self.get_logger().info(f"Calculated initial distance: {self.prev_distance}")
            self.prev_distance_flag = True 
            self.last_progress_check_time = time.time()
            self.progress_check_interval = 1.0  # Check every 1 second
            return
        if msg.feedback.distance_remaining == 0:
            return
        
        self.distance_remaining = msg.feedback.distance_remaining
        
        current_time = time.time()

        if current_time - self.last_progress_check_time >= self.progress_check_interval:

            self.get_logger().info(f'Dist to goal {self.distance_remaining} and prev distance {self.prev_distance}')
            if self.distance_remaining > self.prev_distance:
                self._nav_feedback_counter+=1
                self.get_logger().info(f'Increasing counter {self._nav_feedback_counter}')
                if self._nav_feedback_counter >= self._nav_feedback_limit:
                    self.get_logger().info('Have not made progress within 10 tries, failure')
                    self._trial_result = "FAILURE"
                    self.current_state = SequenceState.FAILED
            else:
                self.prev_distance = self.distance_remaining
                self._nav_feedback_counter = 0

            self.last_progress_check_time = current_time

        
    def terminate(self):
        """Reset the simulation and should kill all the nodes"""
        self.get_logger().info("Terminating function")
        
        if hasattr(self, 'state_timer'):
            self.state_timer.cancel()
        self.get_logger().info("Resetting Gazebo simulation...")
        try:
            # Kill processes in order of dependency
            processes_to_kill = [
                'ros_gz_bridge',
                'parameter_bridge',
                'ign gazebo',
                'ignition gazebo',
                'gz sim',
                'gzserver',
                'gzclient',
                'ignition.launch.py',
                'gz-sim'
            ]
            
            for process in processes_to_kill:
                try:
                    subprocess.run(['pkill', '-f', process], timeout=5)
                    time.sleep(0.5)  # Give processes time to terminate
                except subprocess.TimeoutExpired:
                    # Force kill if normal termination fails
                    subprocess.run(['pkill', '-9', '-f', process], timeout=5)
        
            # Alternative: Kill by process group
            subprocess.run(['pkill', '-f', 'gz'], timeout=5)
            
            self.get_logger().info("Killed ignition processes")
        except Exception as e:
            self.get_logger().error(f"Failed to reset Gazebo: {e}")
        
        rclpy.shutdown()

    def path_coord_to_gazebo_coord(self,x, y):
        # This is from the jackal_timer github repo from dperille (UT-AUSTIN LAB)
        RADIUS=.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)
    def load_barn_path(self, world_num):

        # Load path and convert to gazebo coordinates       
        barn_path = np.load(os.path.expanduser(f'~/ros_ws/BARN_turtlebot/path_files/path_{world_num}.npy'))

        path_msg = Path()
        path_msg.header.frame_id = "map"  # or whatever your map frame is
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for i, element in enumerate(barn_path):
            gazebo_x, gazebo_y = self.path_coord_to_gazebo_coord(element[0], element[1])
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path_msg.header.stamp
            
            pose_stamped.pose.position.x = float(gazebo_x)
            pose_stamped.pose.position.y = float(gazebo_y)
            pose_stamped.pose.position.z = 0.0
            
            # Calculate orientation if not the last point
            if i < len(barn_path) - 1:
                next_element = barn_path[i + 1]
                next_gazebo = self.path_coord_to_gazebo_coord(next_element[0], next_element[1])
                qx, qy, qz, qw = self.calculate_orientation((gazebo_x, gazebo_y), next_gazebo)
                
                pose_stamped.pose.orientation.x = qx
                pose_stamped.pose.orientation.y = qy
                pose_stamped.pose.orientation.z = qz
                pose_stamped.pose.orientation.w = qw
            else:
                # Last point, use previous orientation or default
                pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)

        return path_msg

    def calculate_orientation(self, current_point, next_point):
        """Calculate quaternion orientation from current point to next point"""
        dx = next_point[0] - current_point[0]
        dy = next_point[1] - current_point[1]
        yaw = math.atan2(dy, dx)
        
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        return (0.0, 0.0, qz, qw)  # (qx, qy, qz, qw)

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
