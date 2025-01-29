import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time


class randomMotor(Node):
    def __init__(self):
        super().__init__('random_motor_node')

        self.get_logger().info('Random motor node has started')

    def publish_random_vel(self):

        msg = Twist()

        self.get_logger().info('Publishing motor movement')

def main(args=None):
# Initialize the rclpy library
    rclpy.init(args=args)

# Create and spin the node
    random_motor_node = RandomMotorNode()
    rclpy.spin(random_motor_node)

# Shutdown the node when exiting
    random_motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
