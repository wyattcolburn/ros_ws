import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import Undock
import subprocess

class UndockAndRecord(Node):
    def __init__(self):
        super().__init__('undock_and_record')
        self._action_client = ActionClient(self, Undock, '/undock')

    def undock_robot(self):
        # Wait for the action server to be available
        self.get_logger().info('Waiting for undock action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Undock action server not available!')
            return False

        # Send the undock goal
        goal_msg = Undock.Goal()
        self.get_logger().info('Sending undock goal...')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.undock_goal_response_callback)
        return True

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected!')
            return

        self.get_logger().info('Undock goal accepted. Waiting for result...')
        future = goal_handle.get_result_async()
        future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Undock action completed!')
        self.start_bag_recording()

    def start_bag_recording(self):
        self.get_logger().info('Starting bag recording...')
        # Start the bag recording process
        subprocess.Popen(['ros2', 'bag', 'record', '/scan', '/scan_spoofed', '/tf', '/tf_static', '/odom', '/cmd_vel'])

def main(args=None):
    rclpy.init(args=args)
    undock_and_record = UndockAndRecord()

    if undock_and_record.undock_robot():
        rclpy.spin(undock_and_record)  # Keep the node alive to handle the action callbacks
    else:
        undock_and_record.get_logger().error('Failed to undock robot.')

    undock_and_record.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
