
"""Get init position from launch configurations
"""

from controller_manager_msgs.srv import SwitchController, ListControllers
from rclpy.executors import ExternalShutdownException
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, TwistStamped
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

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.duration import Duration
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime

# imports for rosbag
import signal
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

        # Keep track of results:
        self.trial_results = []
        self._trial_result = None
        self.callback_group = ReentrantCallbackGroup()

        # State management
        self.current_state = SequenceState.IDLE

        self._nav_timeout_counter = 0
        self._nav_timeout_limit = 3

        self._nav_feedback_counter = 0
        self._nav_feedback_limit = 20
        self.prev_distance_flag = False

        self.total_lg = 0
        self.current_lg_counter = 0
        self.current_lg_xy = (0, 0)

        # TF buffer/listener for lookups

        # Publisher for the transformed plan
        self.plan_barn_odom_pub = self.create_publisher(Path, "/plan_barn_odom", 10)
        # Timing
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.clock_running = False
        self.trial_total_time = 0.0
        self.trial_start_time = None  # Time
        self.trial_end_time = None    # Time

        # ===== Velocity stats accumulators =====
        # Commanded (/cmd_vel)
        self._cmd_last_t = None
        self._cmd_total_dt = 0.0
        self._cmd_sum_v_dt = 0.0
        self._cmd_sum_w_dt = 0.0
        self._cmd_sum_v2_dt = 0.0
        self._cmd_sum_w2_dt = 0.0
        self.cmd_avg_lin = None
        self.cmd_avg_ang = None
        self.cmd_rms_lin = None
        self.cmd_rms_ang = None

        # Achieved (/odom)
        self._odom_last_stamp = None  # builtin_interfaces/Time
        self._odom_total_dt = 0.0
        self._odom_sum_v_dt = 0.0
        self._odom_sum_w_dt = 0.0
        self._odom_sum_v2_dt = 0.0
        self._odom_sum_w2_dt = 0.0
        self.odom_avg_lin = None
        self.odom_avg_ang = None
        self.odom_rms_lin = None
        self.odom_rms_ang = None

        # Action clients
        self.undock_client = ActionClient(
            self, Undock, '/undock',
            callback_group=self.callback_group
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.callback_group
        )
        self.follow_client = ActionClient(self, FollowPath, '/follow_path',
                                          callback_group=self.callback_group)
        # Subs/Pubs
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 50)
        self.cmdvel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 50)  # if TwistStamped, swap type

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.path_publisher = self.create_publisher(Path, '/plan_barn', 10)
        self.path_og_publisher = self.create_publisher(Path, '/plan_barn_og', 10)
        self.adaptive_path_publisher = self.create_publisher(Path, '/adaptive_path', 10)
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


        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Current position in map frame
        self.current_map_x = 0.0
        self.current_map_y = 0.0

        self.final_goal_x = None
        self.final_goal_y = None

        self.goal_tolerance_xy = .5

        self.amcl_pose_received = False
        self.pose_stable_count = 0
        self._nav_goal_handle = None  # Track navigation goal handle for cancellation
        self._restart_in_progress = False  # Prevent restart loops

        self.config_radius = None
        self.config_num_valid_obstacles = None
        self.config_offset = None
        self.config_record_csv = None
        self.config_model_path = None
        # gazebo path
        self.gazebo_path = None
        self.gazebo_path_og = None
        self.adaptive_path = None
        # Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('wait_after_undock', 2.0)
        self.declare_parameter('pose_init_delay', 1.0)
        self.declare_parameter('world_num', 0)

        self.collision_detected = False
        self.world_num = self.get_parameter('world_num').value
        self.prev_distance = 0
        self.distance_remaining = 0
        self.goal_x = 0
        self.goal_y = 0

        self.follow_timeout_sec = 300
        self.follow_goal_sent_attempts = 0
        self.FOLLOW_GOAL_MAX_ATTEMPTS = 5
        self._nav_sent = None
        self._nav_start_time = None
        # timer for whole script, max time is 350 after odom starts 
        self.TRIAL_TIMEOUT_AFTER_GOAL_POSE = 370 # after a goal has been given this hits, 
        self.watchdog_timer = self.create_timer(
    1.0, self.watchdog_cb, callback_group=self.callback_group
)
        self.no_odom_timeout_sec    = 1.5     # if no odom for this long → fail
        self.last_odom_steady       = None  
        # ros bag variables
        self.bag_proc = None
        self.bag_outdir = None
        self.bag_topics = ["/odom", "/cmd_vel", "/plan_barn", "/plan_barn_odom", "/tf", "/tf_static", "/scan"]


            # Timer for state machine
        self.state_timer = self.create_timer(
            1.0, self.state_machine_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('ONE SHOT Sequence Node initialized')

        # Start the sequence
        self.start_sequence()

    # =========================
    # Velocity stats helpers
    # =========================
    def _reset_cmd_stats(self):
        self._cmd_last_t = None
        self._cmd_total_dt = 0.0
        self._cmd_sum_v_dt = 0.0
        self._cmd_sum_w_dt = 0.0
        self._cmd_sum_v2_dt = 0.0
        self._cmd_sum_w2_dt = 0.0
        self.cmd_avg_lin = None
        self.cmd_avg_ang = None
        self.cmd_rms_lin = None
        self.cmd_rms_ang = None

    def _reset_odom_stats(self):
        self._odom_last_stamp = None
        self._odom_total_dt = 0.0
        self._odom_sum_v_dt = 0.0
        self._odom_sum_w_dt = 0.0
        self._odom_sum_v2_dt = 0.0
        self._odom_sum_w2_dt = 0.0
        self.odom_avg_lin = None
        self.odom_avg_ang = None
        self.odom_rms_lin = None
        self.odom_rms_ang = None

    def _start_trial_metrics(self):
        self._reset_cmd_stats()
        self._reset_odom_stats()

    def _finalize_cmd_stats(self):
        if self._cmd_total_dt > 0.0:
            self.cmd_avg_lin = self._cmd_sum_v_dt / self._cmd_total_dt
            self.cmd_avg_ang = self._cmd_sum_w_dt / self._cmd_total_dt
            self.cmd_rms_lin = math.sqrt(self._cmd_sum_v2_dt / self._cmd_total_dt)
            self.cmd_rms_ang = math.sqrt(self._cmd_sum_w2_dt / self._cmd_total_dt)
        else:
            self.cmd_avg_lin = self.cmd_avg_ang = 0.0
            self.cmd_rms_lin = self.cmd_rms_ang = 0.0

    def _finalize_odom_stats(self):
        if self._odom_total_dt > 0.0:
            self.odom_avg_lin = self._odom_sum_v_dt / self._odom_total_dt
            self.odom_avg_ang = self._odom_sum_w_dt / self._odom_total_dt
            self.odom_rms_lin = math.sqrt(self._odom_sum_v2_dt / self._odom_total_dt)
            self.odom_rms_ang = math.sqrt(self._odom_sum_w2_dt / self._odom_total_dt)
        else:
            self.odom_avg_lin = self.odom_avg_ang = 0.0
            self.odom_rms_lin = self.odom_rms_ang = 0.0

    def _finalize_all_velocity_stats(self):
        self._finalize_cmd_stats()
        self._finalize_odom_stats()

    # =========================
    # Callbacks
    # =========================
    def cmd_vel_callback(self, msg):
        """
        Time-weighted stats from commanded velocities.
        Works for geometry_msgs/Twist (default). If your publisher uses TwistStamped,
        change the subscriber type and this will still work.
        """
        t_now = self.steady_clock.now()

        # If trial isn't running, just prime the timestamp and return
        if not self.clock_running:
            self._cmd_last_t = t_now
            return

        if self._cmd_last_t is None:
            self._cmd_last_t = t_now
            return

        dt = (t_now - self._cmd_last_t).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            self._cmd_last_t = t_now
            return

        # Support Twist or TwistStamped
        if hasattr(msg, "twist"):
            twist = msg.twist
        else:
            twist = msg

        vx = float(twist.linear.x)
        vy = float(twist.linear.y)
        wz = float(twist.angular.z)

        speed = math.hypot(vx, vy)
        ang_mag = abs(wz)

        self._cmd_sum_v_dt  += speed * dt
        self._cmd_sum_w_dt  += ang_mag * dt
        self._cmd_sum_v2_dt += (speed * speed) * dt
        self._cmd_sum_w2_dt += (wz * wz) * dt
        self._cmd_total_dt  += dt
        self._cmd_last_t     = t_now

    def _accumulate_from_odom(self, odom_msg: Odometry):
        """
        Time-weighted stats from odometry (achieved). Uses header.stamp (ROS time)
        so sim pauses don’t inflate dt.
        """
        if not self.clock_running:
            self._odom_last_stamp = odom_msg.header.stamp
            return

        t = RclpyTime.from_msg(odom_msg.header.stamp)

        if self._odom_last_stamp is None:
            self._odom_last_stamp = odom_msg.header.stamp
            return

        dt = (t - RclpyTime.from_msg(self._odom_last_stamp)).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            self._odom_last_stamp = odom_msg.header.stamp
            return

        vx = float(odom_msg.twist.twist.linear.x)
        vy = float(odom_msg.twist.twist.linear.y)
        wz = float(odom_msg.twist.twist.angular.z)

        speed = math.hypot(vx, vy)
        ang_mag = abs(wz)

        self._odom_sum_v_dt  += speed * dt
        self._odom_sum_w_dt  += ang_mag * dt
        self._odom_sum_v2_dt += (speed * speed) * dt
        self._odom_sum_w2_dt += (wz * wz) * dt
        self._odom_total_dt  += dt
        self._odom_last_stamp = odom_msg.header.stamp

    # =========================
    # Existing logic
    # =========================
    def start_sequence(self):
        """Start the navigation sequence"""
        self.get_logger().info('Starting navigation sequence...')
        self.current_state = SequenceState.INITIALIZING_POSE
        self.retry_count = 0

    def state_machine_callback(self):
        """Main state machine callback with debug logging"""
        self.get_logger().info(
            f"State machine tick: current_state = {self.current_state}")

        if self.current_state == SequenceState.IDLE:
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
            
            ts = datetime.datetime.now().strftime("%m%d_%H%M%S")
            outdir = os.path.expanduser(f"~/ros_ws/new_bt_mlp_asym/world{self.world_num}_{ts}")
            self.start_bag(outdir)
            self.get_logger().info("Starting bag")

            self.gazebo_path = self.load_barn_path(self.world_num)
            self.gazebo_path_og = self.load_barn_path_og(self.world_num)
            path_odom = self.path_to_frame(self.gazebo_path, target_frame="odom")
            if path_odom is not None:
                self.plan_barn_odom_pub.publish(path_odom)
            self.path_publisher.publish(self.gazebo_path)
            self.path_og_publisher.publish(self.gazebo_path_og)
            self.adaptive_path = self.adaptive_barn_path(
                self.gazebo_path,
                min_spacing=0.12,
                max_spacing=0.60,
                k_gain=0.35,
                smoothing=0.5
            )
            self.adaptive_path_publisher.publish(self.adaptive_path)


            self.current_state = SequenceState.NAVIGATING
        elif self.current_state == SequenceState.NAVIGATING:
            self.handle_navigation()
        elif self.current_state == SequenceState.COMPLETED:
            if not hasattr(self, '_completed_logged'):
                self.get_logger().info("Node completed - staying alive")
                self._completed_logged = True
                self._failed_logged = True
                self.record_results()  # end_trial_timer is called inside
                self.stop_bag() 
                self.terminate()
                self.current_state = SequenceState.RESTART


        elif self.current_state == SequenceState.FAILED:
            if not hasattr(self, '_failed_logged'):
                self.get_logger().info("Node failed - staying alive")
                self._failed_logged = True
                self.record_results()  # end_trial_timer is called inside
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
        if not hasattr(self, '_pose_retry_count'):
            self._pose_retry_count = 0
            self._max_retries = 5
            self._retry_interval = 5.0
            self._pose_sent = False
            self._pose_init_time = None
            self.amcl_pose_received = False
            self.pose_stable_count = 0

        should_send_pose = (
            not self._pose_sent or
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
            self.amcl_pose_received = False
            self.pose_stable_count = 0

            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()

            initial_pose.pose.pose.position.x = self.get_parameter('initial_x').value
            initial_pose.pose.pose.position.y = self.get_parameter('initial_y').value
            initial_pose.pose.pose.position.z = 0.0

            yaw = self.get_parameter('initial_yaw').value
            initial_pose.pose.pose.orientation.x = 0.0
            initial_pose.pose.pose.orientation.y = 0.0
            initial_pose.pose.pose.orientation.z = math.sin(yaw/2)
            initial_pose.pose.pose.orientation.w = math.cos(yaw/2)

            covariance_multiplier = 1.0 + (self._pose_retry_count - 1) * 0.5
            initial_pose.pose.covariance = [0.0] * 36
            initial_pose.pose.covariance[0] = 0.25 * covariance_multiplier
            initial_pose.pose.covariance[7] = 0.25 * covariance_multiplier
            initial_pose.pose.covariance[35] = 0.068 * covariance_multiplier

            self.initial_pose_pub.publish(initial_pose)
            self._pose_sent = True
            self._pose_init_time = time.time()

        elif self._pose_sent and self._pose_init_time:
            delay = 13.0
            elapsed = time.time() - self._pose_init_time

            self.get_logger().info(
                f'Waiting for AMCL (attempt {self._pose_retry_count}/{self._max_retries}): '
                f'received={self.amcl_pose_received}, elapsed={elapsed:.1f}s/{delay}s'
            )

            if self.amcl_pose_received and elapsed >= delay:
                self.get_logger().info(
                    f'AMCL pose confirmed after {self._pose_retry_count} attempts, proceeding to navigation...')
                self.current_state = SequenceState.CREATE_PATH
                self._reset_pose_retry_vars()
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
                    self._pose_sent = False
                    self._pose_init_time = None

    def _reset_pose_retry_vars(self):
        self._pose_retry_count = 0
        self._pose_sent = False
        self._pose_init_time = None

    # def handle_navigation(self):
    #     """Handle navigation to goal pose"""
    #     if not hasattr(self, '_nav_sent') or not self._nav_sent:
    #         if not self.navigate_client.wait_for_server(timeout_sec=5.0):
    #             self.get_logger().error('Navigation action server not available')
    #             self._nav_timeout_counter += 1
    #
    #             if self._nav_timeout_counter >= self._nav_timeout_limit:
    #                 self.get_logger().error('Nav server has timed out')
    #                 self._trial_result = "NAV_SERVER_UNAVAILABLE"
    #                 self.current_state = SequenceState.FAILED
    #             return
    #
    #         self.get_logger().info('Sending navigation goal...')
    #
    #         last_pose = self.gazebo_path.poses[-1]
    #         self.get_logger().info(
    #             f'Sending goal to: ({last_pose.pose.position.x}, {last_pose.pose.position.y})')
    #         goal_msg = NavigateToPose.Goal()
    #         goal_msg.pose.header.frame_id = 'map'
    #         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
    #
    #         goal_msg.pose.pose.position.x = self.gazebo_path.poses[-1].pose.position.x
    #         goal_msg.pose.pose.position.y = self.gazebo_path.poses[-1].pose.position.y
    #         goal_msg.pose.pose.position.z = 0.0
    #
    #         goal_msg.pose.pose.orientation.x = self.gazebo_path.poses[-1].pose.orientation.x
    #         goal_msg.pose.pose.orientation.y = self.gazebo_path.poses[-1].pose.orientation.y
    #         goal_msg.pose.pose.orientation.z = self.gazebo_path.poses[-1].pose.orientation.z
    #         goal_msg.pose.pose.orientation.w = self.gazebo_path.poses[-1].pose.orientation.w
    #
    #         self.goal_x = goal_msg.pose.pose.position.x
    #         self.goal_y = goal_msg.pose.pose.position.y
    #
    #         if not self.is_cmd_vel_subscribed():
    #             self.get_logger().warn("cmd_vel has no subscribers — controller likely not active yet")
    #             return
    #
    #         self._nav_future = self.navigate_client.send_goal_async(goal_msg)
    #         self._nav_future.add_done_callback(self.nav_goal_response_callback)
    #         self._nav_sent = True
    #         self._nav_start_time = time.time()
    #     elif time.time() - self._nav_start_time > 300.0:
    #         self.get_logger().warn('Navigation action timed out')
    #         if self._nav_goal_handle:
    #             self._nav_goal_handle.cancel_goal_async()
    #         self._trial_result = "TIMEOUT"
    #         self.current_state = SequenceState.FAILED
    #         self._nav_sent = False
    #
    #     self.final_goal_tracker()
    #
    #     if getattr(self, "_nav_start_time", None) is not None:
    #         elapsed = time.time() - self._nav_start_time
    #         self.get_logger().info(f"current trial time: {elapsed:.2f}s")
    #     else:
    #         self.get_logger().info("current trial time: not started (no nav goal sent yet)")
    #     print(f"current trial time is : {(time.time() - self._nav_start_time)}")
    #     if self.collision_detected:
    #         self.trial_result = "COLLISION"
    #         self.current_state = SequenceState.FAILED
    #
    def handle_navigation(self):
        # Only send once
        if not self._nav_sent:
            if not self.follow_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('FollowPath action server not available')
                self._trial_result = "NAV_SERVER_UNAVAILABLE"
                self.current_state = SequenceState.FAILED
                return

            path_odom = self.path_to_frame(self.gazebo_path, target_frame="odom")
            if path_odom is None or not path_odom.poses:
                self.get_logger().error('No path available to send to FollowPath')
                self._trial_result = "NO_PATH"
                self.current_state = SequenceState.FAILED
                return

            goal = FollowPath.Goal()
            goal.path = path_odom
            goal.controller_id = 'FollowPath'     # must match your YAML key
            goal.goal_checker_id = ''             # default

            self.get_logger().info(f'Sending FollowPath with {len(path_odom.poses)} poses in odom')
            self._follow_future = self.follow_client.send_goal_async(goal)
            self._follow_future.add_done_callback(self._follow_goal_response_cb) # this will set if self._nav_sent is True or false

            return  # let callbacks/watchdog drive the rest

        # Already sent → just report elapsed

        # Optional: lightweight progress checks you already have

        if self.collision_detected:
            self._trial_result = "COLLISION"
            self.current_state = SequenceState.FAILED
    # def handle_navigation(self):
    #
    #     # Wait for controller server
    #     if not self.follow_client.wait_for_server(timeout_sec=5.0):
    #         self.get_logger().error('FollowPath action server not available')
    #         self._trial_result = "NAV_SERVER_UNAVAILABLE"
    #         self.current_state = SequenceState.FAILED
    #         return
    #
    #     # Use your already-computed path; prefer odom-frame for the local controller:
    #     path_odom = self.path_to_frame(self.gazebo_path, target_frame="odom")
    #     if path_odom is None or not path_odom.poses:
    #         self.get_logger().error('No path available to send to FollowPath')
    #         self._trial_result = "NO_PATH"
    #         self.current_state = SequenceState.FAILED
    #         return
    #     
    #     else:
    #         goal = FollowPath.Goal()
    #         goal.path = path_odom
    #         goal.controller_id = 'FollowPath'     # use default (FollowPath in YAML)
    #         goal.goal_checker_id = ''   # use default
    #
    #         self.get_logger().info(f'Sending FollowPath with {len(path_odom.poses)} poses in odom')
    #         self._follow_future = self.follow_client.send_goal_async(goal)
    #         self._follow_future.add_done_callback(self._follow_goal_response_cb)
    #         self._nav_sent = True
    #         self._nav_start_time = time.time()
    #     if time.time() - self._nav_start_time > 300.0:
    #         self.get_logger().warn('Navigation action timed out')
    #     if self._nav_goal_handle:
    #         self._nav_goal_handle.cancel_goal_async()
    #         self._trial_result = "TIMEOUT"
    #         self.current_state = SequenceState.FAILED
    #         self._nav_sent = False
    #     #
    #     self.final_goal_tracker()
    #
    #     if getattr(self, "_nav_start_time", None) is not None:
    #         elapsed = time.time() - self._nav_start_time
    #         self.get_logger().info(f"current trial time: {elapsed:.2f}s")
    #     else:
    #         self.get_logger().info("current trial time: not started (no nav goal sent yet)")
    #         print(f"current trial time is : {(time.time() - self._nav_start_time)}")
    #     if self.collision_detected:
    #         self.trial_result = "COLLISION"
    #         self.current_state = SequenceState.FAILED
    def nav_goal_response_callback(self, future):
        """Callback for navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self._nav_goal_handle = None
            return

        self.get_logger().info('Navigation goal accepted')
        self._nav_goal_handle = goal_handle
        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.nav_result_callback)

    def _follow_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected')
            self._nav_sent = False
            self.follow_goal_sent_attempts +=1 
            if self.follow_goal_sent_attempts > self.FOLLOW_GOAL_MAX_ATTEMPTS: 
                self.get_logger().info(f'FOllowPath goal has been rejected more than allowed attempts, restarting {self.follow_goal_sent_attempts}')
                self.current_state = SequenceState.FAILED
            return
        self.get_logger().info('FollowPath goal accepted')
        self._follow_result_future = goal_handle.get_result_async()
        self._follow_result_future.add_done_callback(self._follow_result_cb)
        self._nav_goal_handle = goal_handle
        self._nav_sent = True

    def _follow_result_cb(self, future):
        try:
            result_msg = future.result()
            status = result_msg.status
            print(f"status is {status}")
    #         if status == 4:  # SUCCESS
    #             self._trial_result = "SUCCESS"
    #             self.current_state = SequenceState.COMPLETED
    #         elif status == 5:  # ABORTED
    #             self._trial_result = "NAV_ABORTED_CONTROLLER_FAIL"
    #             self.current_state = SequenceState.FAILED
    #         elif status == 6:  # CANCELED
    #             self._trial_result = "CANCELLED"
    #             self.current_state = SequenceState.FAILED
    #         else:
    #             self._trial_result = f"UNKNOWN_STATUS_{status}"
    #             self.current_state = SequenceState.FAILED
        except Exception as e:
            print(f"expect block {e}")
    #         self.get_logger().error(f'FollowPath result error: {e}')
    #         self._trial_result = "ERROR"
    #         self.current_state = SequenceState.FAILED
    #     finally:
    #         self._nav_sent = False
    def nav_result_callback(self, future):
        """Callback for navigation result"""
        self.get_logger().info("nav_result_callback called!")

        try:
            if future.cancelled():
                self.get_logger().warn("Navigation goal was cancelled before completion")
                self._trial_result = "CANCELLED"
                # self.current_state = SequenceState.RESTART
                self._nav_sent = False
                return

            if not future.done():
                self.get_logger().warn("Navigation result future is not done yet")
                return

            result_msg = future.result()
            if result_msg is None:
                self.get_logger().error("Navigation result returned None")
                self._trial_result = "ERROR"
                # self.current_state = SequenceState.RESTART
                self._nav_sent = False
                return

            status = result_msg.status
            result = result_msg.result

            self.get_logger().info(f"Nav status: {status}")
            if status == 4:  # SUCCESS
                self.get_logger().info(f'Navigation success: {result}')
                self._trial_result = "SUCCESS"

            elif status == 5:  # ABORTED
                self.get_logger().error(
                    'Navigation aborted: likely planning or controller failure.'
                )
                if self.collision_detected:
                    self._trial_result = "COLLISIONS"
                    self.get_logger().error("Navigation aborted due to collision!")
                else:
                    self._trial_result = "NAV_ABORTED_PLANNING_FAIL"
                    self.get_logger().error("Aborted without collision.")
            elif status == 6:  # CANCELED
                self.get_logger().warn("Navigation was cancelled for trial")
                if self._trial_result != "TIMEOUT":
                    self._trial_result = "FAILURE"
            else:
                self.get_logger().error(
                    f'Navigation failed with unknown status: {status}.')
                # self._trial_result = "NAV_UNKNOWN_FAILURE"

            self.get_logger().info(f"trial_result : {self._trial_result}")
            # self.current_state = SequenceState.COMPLETED
            self._nav_sent = False

        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self._trial_result = "ERROR"
            # self.current_state = SequenceState.FAILED
            self._nav_sent = False

    def amcl_pose_callback(self, msg):
        """Monitor AMCL pose for stability"""
        self.amcl_pose_received = True
        self.pose_stable_count += 1

    def bumper_callback(self, msg):
        """Monitor for bumper collisions"""
        if msg.contacts and self.current_state == SequenceState.NAVIGATING:
            self.get_logger().warn(f"Collision! {len(msg.contacts)} contact")
            # (logs trimmed)
            self.collision_detected = True
            self._trial_result = "COLLISION"
            self.current_state = SequenceState.FAILED

    def is_cmd_vel_subscribed(self):
        """Check if any node is subscribed to /cmd_vel."""
        info = self.get_subscriber_info_by_topic('/cmd_vel')
        return len(info) > 0

    def yaml_reader(self):
        """
        Read configs from YAML. Supports either:
          - Scalar keys: MODEL_PATH, CSV_FILE
          - Or lists:   MODEL_PATHS, CSV_FILES and selects index via $MODEL_INDEX
        Also supports $CONFIG_YAML to override the path (default: ~/ros_ws/config.yaml).
        """
        cfg_path = os.environ.get("CONFIG_YAML", os.path.expanduser('~/ros_ws/config.yaml'))
        with open(cfg_path, "r") as file:
            config = yaml.safe_load(file) or {}

        # Pick model index from env (default 0)
        try:
            idx = int(os.environ.get("MODEL_INDEX", "0"))
        except ValueError:
            idx = 0

        def _select(key_plural, key_singular, default=None):
            if key_plural in config and isinstance(config[key_plural], list) and config[key_plural]:
                # Clamp index to range
                use_idx = max(0, min(idx, len(config[key_plural]) - 1))
                return config[key_plural][use_idx]
            return config.get(key_singular, default)

        # Assign node fields
        self.config_model_path   = _select("MODEL_PATHS", "MODEL_PATH", default="")
        self.config_record_csv   = _select("CSV_FILES",   "CSV_FILE",   default="timer_test/baseline.csv")

        # Other existing keys (unchanged)
        self.config_radius              = config.get("RADIUS", 0.4)
        self.config_num_valid_obstacles = config.get("NUM_VALID_OBSTACLES", 20)
        self.config_offset              = config.get("OFFSET", 1.0)

        self.get_logger().info(
            f"Loaded: MODEL_INDEX={idx}, MODEL_PATH={self.config_model_path}, CSV_FILE={self.config_record_csv}, "
            f"RADIUS={self.config_radius}, NUM={self.config_num_valid_obstacles}, OFFSET={self.config_offset}"
        )
    def record_results(self):
        """Records the results of the current trial into a CSV"""
        self.end_trial_timer()  # finalizes duration + velocity stats
        self.yaml_reader()

        filepath = os.path.join(os.path.expanduser('~'), 'ros_ws', self.config_record_csv)
        print(f" this is filepath {filepath}")
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        new_header = ['timestamp', 'world_num', 'model_path', 'initial_x', 'initial_y', 'initial_yaw',
                      'goal_x', 'goal_y', 'trial_result', 'local_goal_reached', 'num_lg',
                      'RADIUS', 'NUM_VALID_OBSTACLES', 'OFFSET', 'TRIAL_TIME',
                      'CMD_AVG_LIN', 'CMD_AVG_ANG', 'CMD_RMS_LIN', 'CMD_RMS_ANG',
                      'ODOM_AVG_LIN', 'ODOM_AVG_ANG', 'ODOM_RMS_LIN', 'ODOM_RMS_ANG']

        row = [
            timestamp,
            f"world {self.world_num}",
            self.config_model_path,
            self.get_parameter('initial_x').value,
            self.get_parameter('initial_y').value,
            self.get_parameter('initial_yaw').value,
            self.goal_x,
            self.goal_y,
            self._trial_result,
            self.current_lg_counter,
            self.total_lg,
            self.config_radius,
            self.config_num_valid_obstacles,
            self.config_offset,
            f"{self.trial_total_time:.3f}",
            f"{(self.cmd_avg_lin or 0.0):.3f}",
            f"{(self.cmd_avg_ang or 0.0):.3f}",
            f"{(self.cmd_rms_lin or 0.0):.3f}",
            f"{(self.cmd_rms_ang or 0.0):.3f}",
            f"{(self.odom_avg_lin or 0.0):.3f}",
            f"{(self.odom_avg_ang or 0.0):.3f}",
            f"{(self.odom_rms_lin or 0.0):.3f}",
            f"{(self.odom_rms_ang or 0.0):.3f}",
        ]

        if os.path.exists(filepath):
            print(f"File already exists at {filepath}, appending current trial {self._trial_result}")
            with open(filepath, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row)
        else:
            print(f"File {filepath} does not exist, creating file and header")
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(new_header)
                writer.writerow(row)
        return

    def nav_feedback_callback(self, msg):
        """Checks if the robot is making progress towards the goal, terminates if not"""
        if not self.prev_distance_flag:
            initial_x = self.get_parameter('initial_x').value
            initial_y = self.get_parameter('initial_y').value
            goal_x = self.goal_x
            goal_y = self.goal_y

            self.get_logger().info(f"Initial pose: ({initial_x}, {initial_y})")
            self.get_logger().info(f"Goal pose: ({goal_x}, {goal_y})")

            self.prev_distance = math.sqrt((goal_x - initial_x)**2 + (goal_y - initial_y)**2)
            self.get_logger().info(f"Calculated initial distance: {self.prev_distance}")
            self.prev_distance_flag = True
            self.last_progress_check_time = time.time()
            self.progress_check_interval = 1.0
            return

        self.distance_remaining = self.local_goal_tracker()
        self.final_goal_tracker()
        current_time = time.time()

        if current_time - self.last_progress_check_time >= self.progress_check_interval:
            self.get_logger().info(
                f'Dist to goal {self.distance_remaining} and prev distance {self.prev_distance}')
            if self.distance_remaining > self.prev_distance:
                self._nav_feedback_counter += 1
                self.get_logger().info(f'Increasing counter {self._nav_feedback_counter}')
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

    def find_closest_ahead_local_goal(self):
        min_distance = float('inf')
        best_goal_index = self.current_lg_counter
        for i in range(self.current_lg_counter, len(self.gazebo_path.poses)):
            goal_x = self.gazebo_path.poses[i].pose.position.x
            goal_y = self.gazebo_path.poses[i].pose.position.y
            dx = self.current_map_x - goal_x
            dy = self.current_map_y - goal_y
            distance = ((dx*dx) + (dy*dy))**.5
            if distance < min_distance:
                min_distance = distance
                best_goal_index = i
        return best_goal_index, min_distance

    def local_goal_tracker(self):
        best_goal_index, distance_to_best = self.find_closest_ahead_local_goal()
        if best_goal_index > self.current_lg_counter:
            self.get_logger().info(f"Skipping ahead from lg {self.current_lg_counter} to lg {best_goal_index}")
            self.current_lg_counter = best_goal_index
            self.current_lg_xy = (self.gazebo_path.poses[self.current_lg_counter].pose.position.x,
                                  self.gazebo_path.poses[self.current_lg_counter].pose.position.y)
            return distance_to_best

        dx = self.current_map_x - self.current_lg_xy[0]
        dy = self.current_map_y - self.current_lg_xy[1]
        distance_remaining = ((dx*dx) + (dy*dy)**.5)

        if distance_remaining < 0.13:
            self.get_logger().info(f"Passed local goal {self.current_lg_counter}")
            if self.current_lg_counter + 1 < self.total_lg:
                self.current_lg_counter += 1
            if self.current_lg_counter < len(self.gazebo_path.poses):
                self.current_lg_xy = (self.gazebo_path.poses[self.current_lg_counter].pose.position.x,
                                      self.gazebo_path.poses[self.current_lg_counter].pose.position.y)

        return distance_remaining

    def final_goal_tracker(self):

        if self.final_goal_x is None and self.gazebo_path:
            self.final_goal_x = self.gazebo_path.poses[-1].pose.position.x
            self.final_goal_y = self.gazebo_path.poses[-1].pose.position.y

        dx = self.current_map_x - self.final_goal_x
        dy = self.current_map_y - self.final_goal_y
        final_distance = math.hypot(dx, dy)
        print(f"Final Goal Tracker dist : {final_distance}")
        if final_distance < self.goal_tolerance_xy:
            self._trial_result = "SUCCESS"
            self.current_state = SequenceState.COMPLETED

    def odom_callback(self, odom_msg: Odometry):
        if self.current_state != SequenceState.NAVIGATING:
            return
        try:
            v = (odom_msg.twist.twist.linear.x ** 2 + odom_msg.twist.twist.linear.y ** 2) ** 0.5
            moving = v > 0.02  # tune threshold
            self.last_odom_steady = self.steady_clock.now()
            if moving and not self.clock_running:
                self.clock_running = True
                self.trial_start_time = self.steady_clock.now()
                self._start_trial_metrics()  # start velocity accumulators
                self.get_logger().info("TIMER HAS BEGUN")
                self.get_logger().info("TIMER HAS BEGUN")
                self.get_logger().info("TIMER HAS BEGUN")
                self.get_logger().info("TIMER HAS BEGUN")
                self.get_logger().info("TIMER HAS BEGUN")
                self.get_logger().info("TIMER HAS BEGUN")
                self.get_logger().info("TIMER HAS BEGUN")
                self.get_logger().info("TIMER HAS BEGUN")
            # Pose transform to map
            pose_stamped = PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose
            map_pose = self.tf_buffer.transform(pose_stamped, 'map')

            self.current_map_x = map_pose.pose.position.x
            self.current_map_y = map_pose.pose.position.y
            if self.trial_start_time is not None:
                self.get_logger().info(f'Map position: x={self.current_map_x:.2f}, y={self.current_map_y:.2f} current trial time : {self.steady_clock.now() - self.trial_start_time}')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform odom to map: {str(e)}')

        # Accumulate achieved velocity stats regardless of transform success
        self.final_goal_tracker()
        self.local_goal_tracker()
        self._accumulate_from_odom(odom_msg)

    def watchdog_cb(self):
        # Only care during active navigation
        if self.current_state != SequenceState.NAVIGATING:
            return

        now = self.steady_clock.now()

        # Hard trial timeout (steady time, unaffected by sim clock)
        if self.clock_running and self.trial_start_time is not None:
            elapsed = (now - self.trial_start_time).nanoseconds / 1e9
            if elapsed > self.TRIAL_TIMEOUT_AFTER_GOAL_POSE:
                self.get_logger().error(f"Hard timeout ({elapsed:.1f}s) — cancelling nav & failing.")
                if self._nav_goal_handle:
                    try:
                        self._nav_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._trial_result = "TIMEOUT"
                self.current_state = SequenceState.FAILED
                return

        # Odom liveliness (don’t let a stalled odom hide failures)
        if self.last_odom_steady is not None:
            since_odom = (now - self.last_odom_steady).nanoseconds / 1e9
            if since_odom > self.no_odom_timeout_sec:
                self.get_logger().error(f"No /odom for {since_odom:.2f}s — assuming system stalled.")
                if self._nav_goal_handle:
                    try:
                        self._nav_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._trial_result = "NO_ODOM"
                self.current_state = SequenceState.FAILED
    def end_trial_timer(self):
        # Safe no-op if never started
        if self.clock_running and self.trial_start_time is not None:
            end = self.steady_clock.now()
            dur = end - self.trial_start_time
            self.trial_total_time = dur.nanoseconds / 1e9
            self.clock_running = False
            # finalize velocity stats when the trial ends
            self._finalize_all_velocity_stats()
            self.get_logger().info(
                f"[trial] duration = {self.trial_total_time:.3f} s | "
                f"cmd_avg={float(self.cmd_avg_lin or 0):.3f} m/s, {float(self.cmd_avg_ang or 0):.3f} rad/s | "
                f"odom_avg={float(self.odom_avg_lin or 0):.3f} m/s, {float(self.odom_avg_ang or 0):.3f} rad/s"
            )

    def terminate(self):
        """Reset the simulation and should kill all the nodes"""
        self.get_logger().info("Terminating function")
        self.stop_bag() 
        if hasattr(self, 'state_timer'):
            self.state_timer.cancel()
        self.get_logger().info("Resetting Gazebo simulation...")
        try:
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
                    time.sleep(0.5)
                except subprocess.TimeoutExpired:
                    subprocess.run(['pkill', '-9', '-f', process], timeout=5)

            subprocess.run(['pkill', '-f', 'gz'], timeout=5)
            self.get_logger().info("Killed ignition processes")
        except Exception as e:
            self.get_logger().error(f"Failed to reset Gazebo: {e}")

        rclpy.shutdown()

    def path_coord_to_gazebo_coord(self, x, y):
        RADIUS = .075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5
        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift
        return (gazebo_x, gazebo_y)

    def adaptive_barn_path(self, path_msg: Path,
                           min_spacing=0.12, max_spacing=0.60,
                           k_gain=0.35, smoothing=0.5) -> Path:
        import math
        import numpy as np
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped

        def _path_xy(P: Path):
            if not P or not P.poses:
                return np.zeros((0, 2), dtype=float)
            return np.array([(ps.pose.position.x, ps.pose.position.y) for ps in P.poses], dtype=float)

        def _signed_curv(xy: np.ndarray, i: int) -> float:
            n = len(xy)
            if n < 3:
                return 0.0
            i0 = max(0, i - 1)
            i1 = i
            i2 = min(n - 1, i + 1)
            ax, ay = xy[i1] - xy[i0]
            bx, by = xy[i2] - xy[i1]
            la = math.hypot(ax, ay) + 1e-12
            lb = math.hypot(bx, by) + 1e-12
            cross = ax * by - ay * bx
            sinu = cross / (la * lb)
            return sinu / la

        def _target_spacing(k: float) -> float:
            s = max_spacing / (1.0 + k_gain * abs(k))
            return max(min_spacing, min(s, max_spacing))

        def _yaw(p, q):
            return math.atan2(q[1] - p[1], q[0] - p[0])

        def _yaw_to_quat(yaw):
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            return 0.0, 0.0, qz, qw

        xy = _path_xy(path_msg)
        n = len(xy)
        if n == 0:
            self.get_logger().warn("[adaptive_barn_path] input path is empty")
            return Path()
        if n == 1:
            out = Path()
            out.header.frame_id = path_msg.header.frame_id or "map"
            out.header.stamp = self.get_clock().now().to_msg()
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(xy[0, 0])
            ps.pose.position.y = float(xy[0, 1])
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
            return out

        K = np.array([_signed_curv(xy, i) for i in range(n)], dtype=float)

        if 0.0 < smoothing < 1.0 and n >= 2:
            ema = K[0]
            for i in range(n):
                ema = smoothing * ema + (1.0 - smoothing) * K[i]
                K[i] = ema

        kept_idx = [0]
        acc = 0.0
        tgt = _target_spacing(K[0])
        for i in range(1, n):
            seg = float(np.hypot(xy[i, 0] - xy[i - 1, 0], xy[i, 1] - xy[i - 1, 1]))
            acc += seg
            tgt = 0.5 * tgt + 0.5 * _target_spacing(K[i])
            if acc >= tgt:
                kept_idx.append(i)
                acc = 0.0
        if kept_idx[-1] != n - 1:
            kept_idx.append(n - 1)

        xy_keep = xy[kept_idx]

        out = Path()
        out.header.frame_id = path_msg.header.frame_id or "map"
        out.header.stamp = self.get_clock().now().to_msg()

        last_yaw = None
        for i in range(len(xy_keep)):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(xy_keep[i, 0])
            ps.pose.position.y = float(xy_keep[i, 1])
            ps.pose.position.z = 0.0

            if i < len(xy_keep) - 1:
                yaw = _yaw(xy_keep[i], xy_keep[i + 1])
                last_yaw = yaw
            else:
                yaw = last_yaw if last_yaw is not None else 0.0

            qx, qy, qz, qw = _yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            out.poses.append(ps)

        spacings = [math.hypot(xy_keep[i + 1, 0] - xy_keep[i, 0],
                               xy_keep[i + 1, 1] - xy_keep[i, 1])
                    for i in range(len(xy_keep) - 1)]
        if spacings:
            import numpy as np
            self.get_logger().info(
                f"[adaptive_barn_path] kept {len(xy_keep)} of {n} poses; "
                f"spacing min/med/max = {min(spacings):.3f} / {np.median(spacings):.3f} / {max(spacings):.3f} m"
            )

        return out
    def path_to_frame(self, path_msg: Path, target_frame: str = "odom",
                      lookup_timeout_sec: float = 0.25) -> Path | None:
        """
        Transform a nav_msgs/Path into target_frame using the **latest available TF**.
        (Ignores per-pose timestamps to avoid wall/sim-time mismatches.)
        """
        if not path_msg.poses:
            return None

        src_frame = (path_msg.header.frame_id or "map").lstrip("/")
        out = Path()
        out.header = path_msg.header
        out.header.frame_id = target_frame

        # latest TF (Time()=0) so we don't depend on pose/header stamps
        latest = Time()

        try:
            tf_latest = self.tf_buffer.lookup_transform(
                target_frame, src_frame, latest, timeout=Duration(seconds=lookup_timeout_sec)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"[path_to_frame/latest] TF {src_frame}->{target_frame} unavailable: {e}")
            return None

        n_ok = n_skip = 0
        for ps in path_msg.poses:
            try:
                # ps is PoseStamped → use the *stamped* helper
                ps_tf: PoseStamped = do_transform_pose_stamped(ps, tf_latest)
                ps_tf.header.frame_id = target_frame
                out.poses.append(ps_tf)
                n_ok += 1
            except Exception as e:
                self.get_logger().warn(f"[path_to_frame/latest] skipping pose: {e}")
                n_skip += 1

        if not out.poses:
            self.get_logger().error("[path_to_frame/latest] all poses skipped; no output path.")
            return None

        self.get_logger().info(
            f"[path_to_frame/latest] transformed poses: {n_ok} ok, {n_skip} skipped "
            f"(src='{src_frame}' -> '{target_frame}', latest TF)"
        )
        return out



    def load_barn_path_adaptive(self, world_num, base_step=0.20, min_step=0.10, max_step=0.25, smooth_window=5):
        """
        Loads, smooths, and resamples a BARN path with adaptive local-goal spacing
        based on curvature (denser sampling in turns).
        """
        import numpy as np
        import math
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        import os

        # Helper: cumulative arc lengths
        def arc_lengths(xy):
            d = np.diff(xy, axis=0)
            seg = np.hypot(d[:, 0], d[:, 1])
            s = np.concatenate([[0.0], np.cumsum(seg)])
            return s, seg

        # Helper: curvature between three consecutive points
        def local_curvature(p_prev, p_curr, p_next):
            # Triangle method: κ ≈ 4A / (abc)
            a = np.linalg.norm(p_curr - p_prev)
            b = np.linalg.norm(p_next - p_curr)
            c = np.linalg.norm(p_next - p_prev)
            if a * b * c < 1e-8:
                return 0.0
            s = (a + b + c) / 2.0
            area = max(s * (s - a) * (s - b) * (s - c), 0.0)
            area = math.sqrt(area)
            return 4.0 * area / (a * b * c)

        # Load path and convert to Gazebo coordinates
        barn_path = np.load(os.path.expanduser(
            f'~/ros_ws/BARN_turtlebot/path_files/path_{world_num}.npy'
        )).astype(float)
        xy = np.array([self.path_coord_to_gazebo_coord(x, y) for x, y in barn_path], dtype=float)

        # Smooth path for stable curvature estimation
        def smooth_xy(xy, window=5):
            if window < 3 or len(xy) < window:
                return xy
            k = window // 2
            pad = np.pad(xy, ((k, k), (0, 0)), mode='edge')
            kern = np.ones((window, 1)) / window
            xs = np.convolve(pad[:, 0], kern[:, 0], mode='valid')
            ys = np.convolve(pad[:, 1], kern[:, 0], mode='valid')
            return np.stack([xs, ys], axis=1)

        xy = smooth_xy(xy, smooth_window)

        # Build adaptively spaced resampled path
        adaptive_pts = [xy[0]]
        i = 0
        while i < len(xy) - 2:
            kappa = local_curvature(xy[i], xy[i + 1], xy[i + 2])
            # Map curvature to spacing (higher κ → smaller step)
            factor = max(0.3, 1.0 - (kappa * 3.0))  # tweak multiplier as needed
            step = np.clip(base_step * factor, min_step, max_step)
            # Move along arc length until reaching the next step
            j = i + 1
            acc = 0.0
            while j < len(xy):
                seg = np.linalg.norm(xy[j] - xy[j - 1])
                acc += seg
                if acc >= step:
                    adaptive_pts.append(xy[j])
                    break
                j += 1
            i = j

        # Compute yaws for each point
        yaws = []
        for i in range(len(adaptive_pts)):
            if i == len(adaptive_pts) - 1:
                yaw = yaws[-1] if yaws else 0.0
            else:
                dx = adaptive_pts[i + 1][0] - adaptive_pts[i][0]
                dy = adaptive_pts[i + 1][1] - adaptive_pts[i][1]
                yaw = math.atan2(dy, dx)
            yaws.append(yaw)

        # Create ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        now = self.get_clock().now().to_msg()

        for (x, y), yaw in zip(adaptive_pts, yaws):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)

        if path_msg.poses:
            self.current_lg_xy = (
                path_msg.poses[0].pose.position.x,
                path_msg.poses[0].pose.position.y
            )

        self.total_lg = len(path_msg.poses)
        return path_msg
    def load_barn_path(self, world_num, resample_step=0.20, smooth_window=5):
        import numpy as np
        import math
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped

        def arc_lengths(xy):
            d = np.diff(xy, axis=0)
            seg = np.hypot(d[:, 0], d[:, 1])
            s = np.concatenate([[0.0], np.cumsum(seg)])
            return s, seg

        def resample_by_arclen(xy, step=0.12):
            keep = [0]
            for i in range(1, len(xy)):
                if not np.allclose(xy[i], xy[keep[-1]], atol=1e-8):
                    keep.append(i)
            xy = xy[keep]
            if len(xy) < 2:
                return xy
            s, _ = arc_lengths(xy)
            if s[-1] < step:
                return xy
            s_new = np.arange(0.0, s[-1] + 1e-6, step)
            x = np.interp(s_new, s, xy[:, 0])
            y = np.interp(s_new, s, xy[:, 1])
            return np.stack([x, y], axis=1)

        def smooth_xy(xy, window=5):
            if window < 3 or window % 2 == 0 or len(xy) < window:
                return xy
            k = window // 2
            pad = np.pad(xy, ((k, k), (0, 0)), mode='edge')
            kern = np.ones((window, 1)) / window
            xs = np.convolve(pad[:, 0], kern[:, 0], mode='valid')
            ys = np.convolve(pad[:, 1], kern[:, 0], mode='valid')
            return np.stack([xs, ys], axis=1)

        def yaw_from_diffs(dx, dy, last_yaw=None):
            if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                return last_yaw if last_yaw is not None else 0.0
            return math.atan2(dy, dx)

        def yaw_to_quat(yaw):
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            return 0.0, 0.0, qz, qw

        barn_path = np.load(
            os.path.expanduser(f'~/ros_ws/BARN_turtlebot/path_files/path_{world_num}.npy')
        ).astype(float)


        xy = np.array([self.path_coord_to_gazebo_coord(x, y) for x, y in barn_path], dtype=float)
        xy = resample_by_arclen(xy, step=resample_step)
        xy = smooth_xy(xy, window=smooth_window)

        yaws = []
        last_yaw = None
        for i in range(len(xy)):
            if i == len(xy) - 1:
                yaw = last_yaw if last_yaw is not None else 0.0
            else:
                dx = xy[i + 1, 0] - xy[i, 0]
                dy = xy[i + 1, 1] - xy[i, 1]
                yaw = yaw_from_diffs(dx, dy, last_yaw)
            yaws.append(yaw)
            last_yaw = yaw

        path_msg = Path()
        path_msg.header.frame_id = "map"
        now = self.get_clock().now().to_msg()

        for (x, y), yaw in zip(xy, yaws):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = now
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quat(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_msg.poses.append(ps)

        if path_msg.poses:
            self.current_lg_xy = (
                path_msg.poses[0].pose.position.x,
                path_msg.poses[0].pose.position.y
            )

        self.total_lg = len(path_msg.poses)
        return path_msg

    def load_barn_path_og(self, world_num):
        barn_path = np.load(os.path.expanduser(f'~/ros_ws/BARN_turtlebot/path_files/path_{world_num}.npy'))

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        path_subset = barn_path[3:]
        for i, element in enumerate(path_subset):
            gazebo_x, gazebo_y = self.path_coord_to_gazebo_coord(element[0], element[1])

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = path_msg.header.stamp

            pose_stamped.pose.position.x = float(gazebo_x)
            pose_stamped.pose.position.y = float(gazebo_y)
            pose_stamped.pose.position.z = 0.0

            if i < len(path_subset) - 1:
                next_element = path_subset[i + 1]
                next_gazebo = self.path_coord_to_gazebo_coord(next_element[0], next_element[1])
                qx, qy, qz, qw = self.calculate_orientation((gazebo_x, gazebo_y), next_gazebo)

                pose_stamped.pose.orientation.x = qx
                pose_stamped.pose.orientation.y = qy
                pose_stamped.pose.orientation.z = qz
                pose_stamped.pose.orientation.w = qw
            else:
                pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)
        self.current_lg_xy = (path_msg.poses[0].pose.position.x, path_msg.poses[0].pose.position.y)
        return path_msg

    def calculate_orientation(self, current_point, next_point):
        dx = next_point[0] - current_point[0]
        dy = next_point[1] - current_point[1]
        yaw = math.atan2(dy, dx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def restart_trial(self):
        if self._restart_in_progress:
            return
        self._restart_in_progress = True

        self.get_logger().info("Clearing costmaps...")
        try:
            subprocess.run([
                'ros2', 'service', 'call',
                '/global_costmap/clear_entirely_global_costmap',
                'std_srvs/srv/Empty', '{}'
            ], timeout=5)

            subprocess.run([
                'ros2', 'service', 'call',
                '/local_costmap/clear_entirely_local_costmap',
                'std_srvs/srv/Empty', '{}'
            ], timeout=5)

        except Exception as e:
            self.get_logger().error(f"Error clearing costmaps: {e}")

        self.get_logger().info("Resetting Gazebo world...")
        try:
            subprocess.run([
                'ign', 'service', '-s', f'/world/world_50/reset',
                '--reqtype', 'ignition.msgs.WorldControl',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '3000',
                '--req', 'reset_all: true'
            ])
        except Exception as e:
            self.get_logger().error(f"Error resetting sim: {e}")


    def _bag_cmd(self, outdir: str) -> list[str]:
        # Build the rosbag2 command. Add/remove topics as you like.
        cmd = ["ros2", "bag", "record", "-o", outdir]
        # If you want to record everything instead, replace the next line with: cmd += ["-a"]
        cmd += self.bag_topics
        # Optional compression
        return cmd

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
        try:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")


if __name__ == '__main__':
    main()
