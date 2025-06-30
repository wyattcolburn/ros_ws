#!/usr/bin/env python3
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
import time
from enum import Enum
import math
import subprocess

class SequenceState(Enum):
    IDLE = 0
    UNDOCKING = 1
    UNDOCKING_IN_PROGRESS = 2
    WAITING_AFTER_UNDOCK = 3
    INITIALIZING_POSE = 4
    NAVIGATING = 5
    COMPLETED = 6
    FAILED = 7
    RESTART = 8

class ReliableNavigationSequence(Node):
    def __init__(self):
        super().__init__('reliable_navigation_sequence')
        
        # Use reentrant callback group for concurrent action calls

        # Keep track of results:
        self.trial_results = []
        self._trial_result = None
        self.callback_group = ReentrantCallbackGroup()
        
        # State management
        self.current_state = SequenceState.IDLE
        self.num_trials = 10 
        self.current_trial = 0
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

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        self.bumper_sub = self.create_subscription(
            Contacts, '/bumper_contact', self.bumper_callback,
                10)

        self.collision_detected = False

        self.amcl_pose_received = False
        self.pose_stable_count = 0
        self._nav_goal_handle = None  # Track navigation goal handle for cancellationrestart
        self._restart_in_progress = False  # Prevent restart loops
        
        # Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('goal_x', -8.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('wait_after_undock', 2.0)
        self.declare_parameter('pose_init_delay', 1.0)
        
        # Timer for state machine
        self.state_timer = self.create_timer(
            1.0, self.state_machine_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Reliable Navigation Sequence Node initialized')
        
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
        elif self.current_state == SequenceState.NAVIGATING:
            self.handle_navigation()
        elif self.current_state == SequenceState.COMPLETED:
            if not hasattr(self, '_completed_logged'):
                self.get_logger().info("Node completed - staying alive")
                self._completed_logged = True
        elif self.current_state == SequenceState.FAILED:
            if not hasattr(self, '_failed_logged'):
                self.get_logger().info("Node failed - staying alive")
                self._failed_logged = True
        elif self.current_state == SequenceState.RESTART:
            if not self._restart_in_progress:
                self._restart_in_progress = True
                self.get_logger().info("Restarting trial logic...")
                self.restart()


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
            
            if self.amcl_pose_received and elapsed >= delay:
                self.get_logger().info('AMCL pose confirmed, proceeding to navigation...')
                self.current_state = SequenceState.NAVIGATING
                self._pose_sent = False


    def handle_navigation(self):
        """Handle navigation to goal pose"""
        if (self.current_trial >= self.num_trials -1): #minus one because starts at 0
            self.get_logger().info(f"Completed all {self.num_trials}. Shutting down")
            successes = self.trial_results.count("SUCCESS")
            failures = self.trial_results.count("FAILURE")
            timeouts = self.trial_results.count("TIMEOUT")
            collisions = self.trial_results.count("COLLISIONS")

            self.get_logger().info(f"EXPERIMENT COMPLETE:")
            self.get_logger().info(f"  Total trials: {self.num_trials}")
            self.get_logger().info(f"  Successes: {successes}")
            self.get_logger().info(f"  Failures: {failures}")
            self.get_logger().info(f"  Timeouts: {timeouts}")
            self.get_logger().info(f"  Collisions: {collisions}")
            self.get_logger().info(f"  Success rate: {successes/len(self.trial_results)*100:.1f}%")
            self.state_timer.cancel()
            return
        if not hasattr(self, '_nav_sent') or not self._nav_sent:
            if not self.navigate_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation action server not available')
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
            self.current_state = SequenceState.RESTART
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
                trial_result = "SUCCESS"
            else:
                self.get_logger().info(f'Navigation failed: {result}')
                trial_result = "FAILURE"

            # This does ot accout for collisions
            self.get_logger().info(f"Setting trial_result to: {trial_result}")
            self._trial_result = trial_result
            self.current_state = SequenceState.RESTART
            self._nav_sent = False

        except Exception as e:
            self.get_logger().error(f'Navigation result error: {e}')
            self._trial_result = "ERROR"
            self.current_state = SequenceState.RESTART
            self._nav_sent = False
    def restart(self):

        """Restarts the simulator"""

        self._restart_in_progress = False  # Prevent restart loops
        self.collision_detected = False # Restarting
        self.trial_results.append(self._trial_result)
        self.clear_costmaps()
        # Don't set to IDLE, let start_sequence handle the state
        if self.reset_robot_pose():
            #self.dock_robot()
                # Add a small delay before starting
            # self.start_sequence()  # This will set state to UNDOCKING
            self.current_state = SequenceState.INITIALIZING_POSE
            self.current_trial +=1
        else:
            self.get_logger().error("Failed to reset")
            self.current_state = SequenceState.FAILED 
    # def restart(self):
    #     """Restarts the simulator and resets everything for the next trial."""
    #
    #     # Step 1: Cancel any active navigation goal safely
    #     if self._nav_goal_handle:
    #         self.get_logger().info("Canceling existing navigation goal before reset...")
    #         try:
    #             cancel_future = self._nav_goal_handle.cancel_goal_async()
    #             rclpy.spin_until_future_complete(self, cancel_future)
    #             self.get_logger().info("Navigation goal canceled")
    #         except Exception as e:
    #             self.get_logger().warn(f"Failed to cancel nav goal cleanly: {e}")
    #
    #     # Step 2: Invalidate all navigation-related futures and goal handles
    #     self._nav_goal_handle = None
    #     self._nav_future = None
    #     self._get_nav_result_future = None
    #     self._nav_sent = False
    #
    #     # Step 3: Reset simulation pose
    #     if not self.reset_robot_pose():
    #         self.get_logger().error("Failed to reset robot pose")
    #         self.current_state = SequenceState.FAILED
    #         return
    #
    #     self.get_logger().info("Robot pose reset. Attempting to reset controller...")
    #
    #     # Step 4: Reset the diffdrive_controller safely
    #     if not self.reset_diffdrive_controller():
    #         self.get_logger().error("Failed to reset diffdrive_controller")
    #         self.current_state = SequenceState.FAILED
    #         return
    #
    #     self.get_logger().info("Navigation action state cleared and controller reset")
    #
    #     # Step 5: Rebuild the goal pose (identical values, new instance)
    #     goal_msg = NavigateToPose.Goal()
    #     goal_msg.pose.header.frame_id = 'map'
    #     goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
    #
    #     goal_msg.pose.pose.position.x = self.get_parameter('goal_x').value
    #     goal_msg.pose.pose.position.y = self.get_parameter('goal_y').value
    #     goal_msg.pose.pose.position.z = 0.0
    #
    #     goal_yaw = self.get_parameter('goal_yaw').value
    #     goal_msg.pose.pose.orientation.x = 0.0
    #     goal_msg.pose.pose.orientation.y = 0.0
    #     goal_msg.pose.pose.orientation.z = math.sin(goal_yaw / 2)
    #     goal_msg.pose.pose.orientation.w = math.cos(goal_yaw / 2)
    #
    #     self._next_goal_pose = goal_msg
    #
    #     # Step 6: Advance trial and reset to pose initialization
    #     self.current_trial += 1
    #     self.current_state = SequenceState.INITIALIZING_POSE
    #     self._pose_sent = False          # force a new /initialpose publish
    #     self.amcl_pose_received = False  # force AMCL wait again
    #     self._restart_in_progress = False
    def amcl_pose_callback(self, msg):
        """Monitor AMCL pose for stability"""
        self.amcl_pose_received = True
        self.pose_stable_count += 1
    def reset_robot_pose(self):
        """Restarts the simulator by resetting robot pose to origin with correct docked orientation"""
        try:
            # Convert 3.141 radians (180°) to quaternion
            # For yaw rotation: z = sin(yaw/2), w = cos(yaw/2)
            import math
            yaw = 3.141
            z = math.sin(yaw/2)  # ≈ 1.0
            w = math.cos(yaw/2)  # ≈ 0.0
            
            cmd = [
                'ign', 'service', '-s', '/world/maze/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "turtlebot4", position: {{x: -.25, y: -0.0, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {z:.6f}, w: {w:.6f}}}'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info("Successfully reset robot pose to docked orientation")
                return True
            else:
                self.get_logger().error(f"Failed to reset pose: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("Service call timed out")
            return False
        except Exception as e:
            self.get_logger().error(f"Error calling service: {str(e)}")
            return False


    def reset_diffdrive_controller(self):
        """Safely reset diffdrive_controller (deactivate then activate)"""
        client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("Waiting for /switch_controller service...")

        if not self.is_diffdrive_active():
            self.get_logger().info("diffdrive_controller already inactive — proceeding to activate")
        else:
            deactivate_req = SwitchController.Request()
            deactivate_req.deactivate_controllers = ['diffdrive_controller']
            deactivate_req.activate_controllers = []
            deactivate_req.strictness = SwitchController.Request.BEST_EFFORT
            future = client.call_async(deactivate_req)
            rclpy.spin_until_future_complete(self, future)

            if not future.result() or not future.result().ok:
                self.get_logger().error("Failed to deactivate diffdrive_controller")
                return False

            self.get_logger().info("Successfully deactivated diffdrive_controller")
            time.sleep(1.0)
          # Attempt to activate controller
            activate_req = SwitchController.Request()
            activate_req.activate_controllers = ['diffdrive_controller']
            activate_req.deactivate_controllers = []
            activate_req.strictness = SwitchController.Request.BEST_EFFORT

            future = client.call_async(activate_req)
            rclpy.spin_until_future_complete(self, future)

            if not future.result() or not future.result().ok:
                self.get_logger().error("Failed to activate diffdrive_controller")
                return False

            # Wait for controller to become active again
            for _ in range(5):  # retry up to 5 seconds
                if self.is_diffdrive_active():
                    self.get_logger().info("Successfully activated diffdrive_controller")
                    return True
                self.get_logger().warn("Waiting for diffdrive_controller to become active...")
                time.sleep(1.0)

            self.get_logger().error("diffdrive_controller never became active after activation")
            return False
    def bumper_callback(self, msg):
        """Monitor for bumper collisions"""
        if msg.contacts: # collision detected
            self.get_logger().warn(f"Collision! {len(msg.contacts)} contact")

            for i, contact in enumerate(msg.contacts):
                self.get_logger().info(f"Collision {i} {contact.collision1.name} hit {contact.collision2.name}")
                self.get_logger().info(f"This collision occured at {contact.positions}")

            self.collision_detected = True
            self._trial_result = "COLLISIONS"
            self.current_state = SequenceState.RESTART

    def clear_costmaps(self):
        """Clears both global and local costmaps"""
        try:
        # Clear global costmap
            global_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            if global_client.wait_for_service(timeout_sec=2.0):
                global_req = ClearEntireCostmap.Request()
                global_future = global_client.call_async(global_req)
                rclpy.spin_until_future_complete(self, global_future)
                self.get_logger().info("Global costmap cleared")
            
            # Clear local costmap  
            local_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
            if local_client.wait_for_service(timeout_sec=2.0):
                local_req = ClearEntireCostmap.Request()
                local_future = local_client.call_async(local_req)
                rclpy.spin_until_future_complete(self, local_future)
                self.get_logger().info("Local costmap cleared")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to clear costmaps: {e}")
    def is_diffdrive_active(self):
        """Retries lookup of diffdrive_controller and checks if it's active."""
        client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        retries = 3
        for attempt in range(retries):
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("Waiting for list_controllers service...")

            req = ListControllers.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            if future.result():
                for controller in future.result().controller:
                    if controller.name == 'diffdrive_controller':
                        self.get_logger().info(f"diffdrive_controller state: {controller.state}")
                        return controller.state == 'active'

            self.get_logger().warn(f"diffdrive_controller not found (attempt {attempt + 1}/{retries})")
            time.sleep(1.0)

        self.get_logger().error("diffdrive_controller not found after retries")
        return False
         
    def is_cmd_vel_subscribed(self):
        """Check if any node is subscribed to /cmd_vel."""
        info = self.get_publishers_info_by_topic('/cmd_vel')
        return len(info) > 0
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
