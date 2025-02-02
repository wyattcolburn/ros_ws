import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock


class UndockActionClient(Node):
    def __init__(self):
        super().__init__('undock_action_client')
        self.client = ActionClient(self, Undock, '/undock')
        self.get_logger().info("Waiting for Undock action server...")
        self.client.wait_for_server()
        self.get_logger().info("Undock action server found! Sending request...")
        self.send_goal()

    def send_goal(self):
        goal_msg = Undock.Goal()
        future = self.client.send_goal_async(goal_msg)

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock action was not accepted.")
            rclpy.shutdown()
            return

        self.get_logger().info("Undock action accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info("Undocking complete! Shutting down node.")
        else:
            self.get_logger().error("Undocking failed.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = UndockActionClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

