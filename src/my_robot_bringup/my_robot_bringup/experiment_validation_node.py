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
import math
import subprocess
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import ClearEntireCostmap

class SequenceState(Enum):
    IDLE = 0
    UNDOCKING = 1
    UNDOCKING_IN_PROGRESS = 2  # ← Add this new state
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
        
 # Create service clients for better reliability
        self.controller_state_client = self.create_client(
            ChangeState, '/controller_server/change_state'
        )
        self.planner_state_client = self.create_client(
            ChangeState, '/planner_server/change_state'
        )
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap'
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
        
        self.amcl_pose_received = False
        self.pose_stable_count = 0
        
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
            1.0, self.state_machine_callback,
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
            self.current_state = SequenceState.INITIALIZING_POSE
        elif self.current_state == SequenceState.INITIALIZING_POSE:
            self.get_logger().info("Setting init position")
            self.handle_pose_initialization()    
        elif self.current_state == SequenceState.NAVIGATING:
            self.get_logger().info('Navigating')
            self.handle_navigation()
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
        elif self.current_state == SequenceState.RESTART:
            self.restart(None)

    def handle_undocking(self):
        """Handle undocking phase - only sends command once"""
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Undock action server not available')
            #self.retry_or_fail()
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
            #self.retry_or_fail()

    def undock_goal_response_callback(self, future):
        """Callback for undock goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected')
            #self.retry_or_fail()
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
    def handle_pose_initialization(self):
        """Handle initial pose setting"""
        if not hasattr(self, '_pose_sent') or not self._pose_sent:
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
            initial_pose.pose.pose.orientation.z = math.sin(yaw/2) # sin(yaw/2) for simple case
            initial_pose.pose.pose.orientation.w = math.cos(yaw/2) # cos(yaw/2) for simple case
            
            # Set covariance (you may want to adjust these values)
            initial_pose.pose.covariance = [0.0] * 36
            initial_pose.pose.covariance[0] = 0.25   # x
            initial_pose.pose.covariance[7] = 0.25   # y
            initial_pose.pose.covariance[35] = 0.068 # yaw
            
            self.initial_pose_pub.publish(initial_pose)
            self._pose_sent = True
            self._pose_init_time = time.time()
            # self.amcl_pose_received = False
            # self.pose_stable_count = 0
            
          # Wait a bit for pose to be processed
        elif hasattr(self, '_pose_init_time'):
            delay = 10.0
            elapsed = time.time() - self._pose_init_time
            
            self.get_logger().info(f'Waiting for AMCL: received={self.amcl_pose_received}, elapsed={elapsed:.1f}s/{delay}s')
            
            # Proceed when AMCL has received pose AND minimum delay has passed
            if self.amcl_pose_received and elapsed >= delay:
                self.get_logger().info('AMCL pose confirmed, proceeding to navigation...')
                self.current_state = SequenceState.NAVIGATING
                self._pose_sent = False

    def handle_navigation(self):
        """Handle navigation to goal pose"""
        if not hasattr(self, '_nav_sent') or not self._nav_sent:
            if not self.navigate_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation action server not available')
                self.retry_or_fail()
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
            goal_msg.pose.pose.orientation.z = 0.0  # sin(yaw/2) for simple case
            goal_msg.pose.pose.orientation.w = 1.0  # cos(yaw/2) for simple case
            
            self._nav_future = self.navigate_client.send_goal_async(goal_msg)
            self._nav_future.add_done_callback(self.nav_goal_response_callback)
            self._nav_sent = True
            self._nav_start_time = time.time()

        # Check for timeout (adjust as needed for your environment)
        elif time.time() - self._nav_start_time > 120.0:  # 2 minute timeout
            self.get_logger().warn('Navigation action timed out')
            self.retry_or_fail()

    def nav_goal_response_callback(self, future):
        """Callback for navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            #self.retry_or_fail()
            return

        self.get_logger().info('Navigation goal accepted')
        self._get_nav_result_future = goal_handle.get_result_async()
        self._get_nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Callback for navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed with result: {result}')
        
        self.current_state = SequenceState.RESTART
        self._nav_sent = False  # Reset for potential retry

    def restart(self, result):
        """Restarts the simulator

        # Set robot pose to origin (adjust namespace if needed)
        ign service -s /world/maze/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 1000 --req 'name: "turtlebot4", position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
        """
        self.get_logger().info(f"Trial x was a {result}")

        self.current_state = SequenceState.IDLE
        if (self.reset_robot_pose()):
            if (self.restart_controller_server()):
                self.start_sequence()
            else:
                self.get_logger().error("Failed to reset controller server")
                self.current_state = SequenceState.FAILED
        else:
            self.get_logger().error("Failed to reset")
            self.current_state = SequenceState.FAILED 

    def amcl_pose_callback(self, msg):
        """Monitor AMCL pose for stability"""
        self.amcl_pose_received = True
        # You could also check if pose is "stable" by comparing with previous poses
        self.pose_stable_count += 1


    def reset_robot_pose(self):
            """Restarts the simulator by resetting robot pose to origin"""
            try:
                cmd = [
                    'ign', 'service', '-s', '/world/maze/set_pose',
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', 'name: "turtlebot4", position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0:
                    self.get_logger().info("Successfully reset robot pose")
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


    def restart_controller_server(self):
        """Restart navigation stack using service clients"""
        try:
            # 1. Cancel active navigation
            self.get_logger().info("Canceling active navigation...")
            self.navigate_client.cancel_all_goals()
            
            # 2. Clear costmaps
            if not self._clear_costmaps():
                self.get_logger().warn("Failed to clear costmaps, continuing anyway...")
            
            # 3. Restart lifecycle nodes
            if self._restart_lifecycle_node('controller_server') and \
               self._restart_lifecycle_node('planner_server'):
                self.get_logger().info("Navigation stack restarted successfully")
                return True
            else:
                self.get_logger().error("Failed to restart navigation components")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Failed to restart navigation stack: {e}")
            return False

    def _clear_costmaps(self):
        """Clear costmaps using service clients"""
        success = True
        
        # Clear global costmap
        if self.clear_global_costmap_client.wait_for_service(timeout_sec=2.0):
            request = ClearEntireCostmap.Request()
            future = self.clear_global_costmap_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None:
                self.get_logger().warn("Failed to clear global costmap")
                success = False
        else:
            self.get_logger().warn("Global costmap clear service not available")
            success = False
        
        # Clear local costmap
        if self.clear_local_costmap_client.wait_for_service(timeout_sec=2.0):
            request = ClearEntireCostmap.Request()
            future = self.clear_local_costmap_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None:
                self.get_logger().warn("Failed to clear local costmap")
                success = False
        else:
            self.get_logger().warn("Local costmap clear service not available")
            success = False
            
        return success

    def _restart_lifecycle_node(self, node_name):
        """Restart a lifecycle node using service client"""
        client = getattr(self, f'{node_name.replace("_", "_")}_state_client')
        
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'{node_name} service not available')
            return False
        
        try:
            # Deactivate
            deactivate_req = ChangeState.Request()
            deactivate_req.transition.id = 3  # TRANSITION_DEACTIVATE
            
            future = client.call_async(deactivate_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None or not future.result().success:
                self.get_logger().error(f'Failed to deactivate {node_name}')
                return False
            
            # Brief pause
            time.sleep(0.5)
            
            # Activate
            activate_req = ChangeState.Request()
            activate_req.transition.id = 4  # TRANSITION_ACTIVATE
            
            future = client.call_async(activate_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None or not future.result().success:
                self.get_logger().error(f'Failed to activate {node_name}')
                return False
            
            self.get_logger().info(f'Successfully restarted {node_name}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error restarting {node_name}: {e}')
            return False
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


"""
pntroller_server-1] mod factor is 1.49182
[controller_server-1] [INFO] [1750726735.801638752] [controller_server]: Final commands after modulation (lin, ang): 0.270, -0.133
[controller_server-1] [INFO] [1750726735.801653425] [controller_server]: Final clamped commands (lin, ang): 0.270, -0.133
[planner_server-3] [INFO] [1750726735.820640924] [global_costmap.global_costmap]: Received request to clear entirely the global_costmap
[controller_server-1] [INFO] [1750726735.850750044] [controller_server]: Passing new path to controller.
[controller_server-1] [ERROR] [1750726735.851019768] [controller_server]: Invalid path, Path is empty.
[controller_server-1] [WARN] [1750726735.851070596] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[controller_server-1] [INFO] [1750726735.880884418] [local_costmap.local_costmap]: Received request to clear entirely the local_costmap
[controller_server-1] [INFO] [1750726735.882182558] [controller_server]: Received a goal, begin computing control effort.
[controller_server-1] [ERROR] [1750726735.882273495] [controller_server]: Invalid path, Path is empty.
[controller_server-1] [INFO] [1750726735.882320922] [controller_server]: Receiving Input from Middle man...
[controller_server-1] [WARN] [1750726735.882322256] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[bt_navigator-5] [ERROR] [1750726735.921096240] [bt_navigator_navigate_to_pose_rclcpp_node]: Failed to get result for compute_path_to_pose in node halt!
 maybe this error message can be used to record failures
"""
