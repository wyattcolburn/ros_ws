import os
import time
import subprocess
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Undock

class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')
        
        # Declare and retrieve parameters
        self.declare_parameter('input_dkr', '/home/wyattcolburn/ros_ws/utils/ten_thou/')
        self.declare_parameter('use_sim_time', False)
        self.input_dkr = self.get_parameter('input_dkr').value
        
        # Setup publisher for Twist commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Load CSV file with commands
        csv_file_path = os.path.join(self.input_dkr, "cmd_vel_output.csv")
        self.cmd_data = pd.read_csv(csv_file_path)
        self.index = 0
        self.max_index = len(self.cmd_data)
        self.get_logger().info(f"Loaded {self.max_index} commands from {csv_file_path}")
        
        # Prepare the undock action client
        self._action_client = ActionClient(self, Undock, '/undock')
        
        # Timer for publishing commands will be created later (after undocking)
        self.timer = None
        
        # Begin the undock sequence
        self.send_undock_goal()

    def send_undock_goal(self):
        goal_msg = Undock.Goal()
        self.get_logger().info('Waiting for undock action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Sending undock goal...')
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.undock_feedback_callback)
        future.add_done_callback(self.undock_goal_response_callback)
        
    def undock_feedback_callback(self, feedback_msg):
        self.get_logger().info('Undocking in progress...')
    
    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Undock goal rejected!')
            return
        self.get_logger().info('Undock goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_done_callback)
    
    def undock_done_callback(self, future):
        self.get_logger().info('Undocking complete!')
        self.start_rosbag_recording()
        # Now that undocking is done, start publishing the CSV commands.
        self.timer = self.create_timer(0.1, self.publish)

    def start_rosbag_recording(self):
        self.get_logger().info('Starting rosbag recording...')
        time_arg = "--use-sim-time" if self.get_parameter('use_sim_time').value else ""
        # Note: You might want to add additional arguments or error handling here.
        subprocess.Popen(['ros2', 'bag', 'record', '/scan', '/scan_spoofed', '/tf', 
                          '/tf_static', '/odom', '/cmd_vel', '/clock'])
    
    def publish(self):
        if self.index < self.max_index:
            msg = Twist()
            values = self.cmd_data.iloc[self.index]
            msg.linear.x = values[0]
            msg.angular.z = values[1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published command {self.index+1}/{self.max_index}')
            self.index += 1
        else:
            self.get_logger().info('Published all commands. Stopping timer.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    combined_node = CombinedNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(combined_node, executor=executor)
    combined_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

