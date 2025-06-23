"""
This node is responsible for undocking, initing position, setting goal pose, tracking if success or failure


"""
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from irobot_create_msgs.action import Undock
from nav2_msgs.action import NavigateToPose
import time
from enum import Enum


class SequenceState(Enum):
    IDLE = 0
    UNDOCKING = 1
    UNDOCKING_IN_PROGRESS = 2  # ← Add this new state
    WAITING_AFTER_UNDOCK = 3
    INITIALIZING_POSE = 4
    NAVIGATING = 5
    COMPLETED = 6
    FAILED = 7


class ReliableNavigationSequence(Node):
    def __init__(self):
        super().__init__('reliable_navigation_sequence')
        
        # Use reentrant callback group for concurrent action calls
        self.callback_group = ReentrantCallbackGroup()
        
        # State management
        self.current_state = SequenceState.IDLE
        self.retry_count = 0
        self.max_retries = 3
        
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
        
        # Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('goal_x', -8.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('wait_after_undock', 2.0)  # seconds
        self.declare_parameter('pose_init_delay', 1.0)    # seconds
        
        # Timer for state machine
        self.state_timer = self.create_timer(
            0.5, self.state_machine_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Reliable Navigation Sequence Node initialized')
        
        # Start the sequence
        self.start_sequence()

    def start_sequence(self):
        """Start the navigation sequence"""
        self.get_logger().info('Starting navigation sequence...')
        self.current_state = SequenceState.UNDOCKING
        self.retry_count = 0


    def state_machine_callback(self):
        """Main state machine callback with debug logging"""
        # Add debug logging to see what's happening
        self.get_logger().info(f"State machine tick: current_state = {self.current_state}")
        
        if self.current_state == SequenceState.UNDOCKING:
            self.get_logger().info("Calling handle_undocking()")
            self.handle_undocking()
        elif self.current_state == SequenceState.UNDOCKING_IN_PROGRESS:
            self.get_logger().info("In UNDOCKING_IN_PROGRESS state")
            self.handle_undocking_in_progress()
        elif self.current_state == SequenceState.WAITING_AFTER_UNDOCK:
            self.get_logger().info("Undocking sequence completed successfully!")
            self.current_state = SequenceState.COMPLETED
        elif self.current_state == SequenceState.COMPLETED:
            # Only log this once
            if not hasattr(self, '_completed_logged'):
                self.get_logger().info("Node completed - staying alive")
                self._completed_logged = True
        elif self.current_state == SequenceState.FAILED:
            # Only log this once  
            if not hasattr(self, '_failed_logged'):
                self.get_logger().info("Node failed - staying alive")
                self._failed_logged = True

    def handle_undocking(self):
        """Handle undocking phase - only sends command once"""
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Undock action server not available')
            self.retry_or_fail()
            return
        
        self.get_logger().info('Sending undock command...')
        goal_msg = Undock.Goal()
        self._undock_future = self.undock_client.send_goal_async(goal_msg)
        self._undock_future.add_done_callback(self.undock_goal_response_callback)
        
        # Immediately transition to "in progress" state
        self.current_state = SequenceState.UNDOCKING_IN_PROGRESS
        self.get_logger().info('Moving to new state, waiting for response of undock goal')
        self._undock_start_time = time.time()

    def handle_undocking_in_progress(self):
        """Handle timeout while undocking is in progress"""
        if time.time() - self._undock_start_time > 60.0:  # 30 second timeout
            self.get_logger().warn('Undock action timed out')
            self.retry_or_fail()

    def undock_goal_response_callback(self, future):
        """Callback for undock goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected')
            self.retry_or_fail()
            return

        self.get_logger().info('Undock goal accepted, waiting for completion...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        """Callback for undock result"""
        result = future.result().result
        self.get_logger().info(f'Undock completed with result: {result}')
        
        # Move to next state
        self.current_state = SequenceState.WAITING_AFTER_UNDOCK
        self._wait_start_time = time.time()

    def retry_or_fail(self):
        """Handle retries or failure"""
        self.retry_count += 1
        if self.retry_count < self.max_retries:
            self.get_logger().warn(f'Retrying... Attempt {self.retry_count + 1}/{self.max_retries}')
            # Reset to initial undocking state
            self.current_state = SequenceState.UNDOCKING
        else:
            self.get_logger().error(f'Max retries ({self.max_retries}) exceeded')
            self.current_state = SequenceState.FAILED
    # def handle_waiting_after_undock(self):
    #     """Wait a bit after undocking before initializing pose"""
    #     wait_time = self.get_parameter('wait_after_undock').value
    #     if time.time() - self._wait_start_time >= wait_time:
    #         self.current_state = SequenceState.INITIALIZING_POSE
    #
    # def handle_pose_initialization(self):
    #     """Handle initial pose setting"""
    #     if not hasattr(self, '_pose_sent') or not self._pose_sent:
    #         self.get_logger().info('Setting initial pose...')
    #         
    #         initial_pose = PoseWithCovarianceStamped()
    #         initial_pose.header.frame_id = 'map'
    #         initial_pose.header.stamp = self.get_clock().now().to_msg()
    #         
    #         # Set position
    #         initial_pose.pose.pose.position.x = self.get_parameter('initial_x').value
    #         initial_pose.pose.pose.position.y = self.get_parameter('initial_y').value
    #         initial_pose.pose.pose.position.z = 0.0
    #         
    #         # Set orientation (convert yaw to quaternion)
    #         yaw = self.get_parameter('initial_yaw').value
    #         initial_pose.pose.pose.orientation.x = 0.0
    #         initial_pose.pose.pose.orientation.y = 0.0
    #         initial_pose.pose.pose.orientation.z = 0.0  # sin(yaw/2) for simple case
    #         initial_pose.pose.pose.orientation.w = 1.0  # cos(yaw/2) for simple case
    #         
    #         # Set covariance (you may want to adjust these values)
    #         initial_pose.pose.covariance = [0.0] * 36
    #         initial_pose.pose.covariance[0] = 0.25   # x
    #         initial_pose.pose.covariance[7] = 0.25   # y
    #         initial_pose.pose.covariance[35] = 0.068 # yaw
    #         
    #         self.initial_pose_pub.publish(initial_pose)
    #         self._pose_sent = True
    #         self._pose_init_time = time.time()
    #         
    #     # Wait a bit for pose to be processed
    #     delay = self.get_parameter('pose_init_delay').value
    #     if time.time() - self._pose_init_time >= delay:
    #         self.current_state = SequenceState.NAVIGATING
    #         self._pose_sent = False  # Reset for potential retry
    #
    # def handle_navigation(self):
    #     """Handle navigation to goal pose"""
    #     if not hasattr(self, '_nav_sent') or not self._nav_sent:
    #         if not self.navigate_client.wait_for_server(timeout_sec=5.0):
    #             self.get_logger().error('Navigation action server not available')
    #             self.retry_or_fail()
    #             return
    #         
    #         self.get_logger().info('Sending navigation goal...')
    #         
    #         goal_msg = NavigateToPose.Goal()
    #         goal_msg.pose.header.frame_id = 'map'
    #         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
    #         
    #         # Set goal position
    #         goal_msg.pose.pose.position.x = self.get_parameter('goal_x').value
    #         goal_msg.pose.pose.position.y = self.get_parameter('goal_y').value
    #         goal_msg.pose.pose.position.z = 0.0
    #         
    #         # Set goal orientation
    #         goal_yaw = self.get_parameter('goal_yaw').value
    #         goal_msg.pose.pose.orientation.x = 0.0
    #         goal_msg.pose.pose.orientation.y = 0.0
    #         goal_msg.pose.pose.orientation.z = 0.0  # sin(yaw/2) for simple case
    #         goal_msg.pose.pose.orientation.w = 1.0  # cos(yaw/2) for simple case
    #         
    #         self._nav_future = self.navigate_client.send_goal_async(goal_msg)
    #         self._nav_future.add_done_callback(self.nav_goal_response_callback)
    #         self._nav_sent = True
    #         self._nav_start_time = time.time()
    #
    #     # Check for timeout (adjust as needed for your environment)
    #     elif time.time() - self._nav_start_time > 120.0:  # 2 minute timeout
    #         self.get_logger().warn('Navigation action timed out')
    #         self.retry_or_fail()
    #
    # def nav_goal_response_callback(self, future):
    #     """Callback for navigation goal response"""
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().error('Navigation goal rejected')
    #         self.retry_or_fail()
    #         return
    #
    #     self.get_logger().info('Navigation goal accepted')
    #     self._get_nav_result_future = goal_handle.get_result_async()
    #     self._get_nav_result_future.add_done_callback(self.nav_result_callback)
    #
    # def nav_result_callback(self, future):
    #     """Callback for navigation result"""
    #     result = future.result().result
    #     self.get_logger().info(f'Navigation completed with result: {result}')
    #     
    #     self.current_state = SequenceState.COMPLETED
    #     self._nav_sent = False  # Reset for potential retry
    #

def main(args=None):
    rclpy.init(args=args)
    
    node = ReliableNavigationSequence()
    
    # Use MultiThreadedExecutor for concurrent action handling
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
