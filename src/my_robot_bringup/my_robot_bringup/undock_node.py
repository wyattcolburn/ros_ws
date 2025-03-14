import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock
import subprocess

class UndockActionClient(Node):
    def __init__(self):
        super().__init__('undock_action_client')
        self._action_client = ActionClient(self, Undock, '/undock')

    def send_goal(self):
        goal_msg = Undock.Goal()
        self._action_client.wait_for_server()
        self.get_logger().info('Sending undock goal...')
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Undocking in progress...')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Undock goal rejected!')
            return
        self.get_logger().info('Undock goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_done_callback)

    def undock_done_callback(self, future):
        self.get_logger().info('Undocking complete!')
        
        self.get_logger().info("***********************************************")
        # Start bag recording after undocking
        self.get_logger().info('Starting rosbag recording...')
        
        time_arg = "--use-sim-time" if self.get_parameter('use_sim_time').value else ""
    
        subprocess.Popen(['ros2', 'bag', 'record', '/scan', '/scan_spoofed', '/tf', '/tf_static', '/odom', '/cmd_vel', '/clock'])
        

        ## Start random_motor node after undocking
        #self.get_logger().info('Starting random_motor node...')
        #subprocess.Popen(['ros2', 'run', 'my_robot_bringup', 'random_motor'])

def main(args=None):
    rclpy.init(args=args)
    undock_client = UndockActionClient()
    undock_client.send_goal()
    rclpy.spin(undock_client)
    undock_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

