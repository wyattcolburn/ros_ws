import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LiDARTest(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/diff_drive/scan',
            self.lidar_callback,
            10)
        self.subscription

    def lidar_callback(self, msg):
        non_zero_ranges = [value for value in msg.ranges if value > 0.0]
        if non_zero_ranges:
            self.get_logger().info(f"Non-zero LiDAR ranges: {non_zero_ranges}")
        else:
            self.get_logger().info("No non-zero ranges found.")

def main(args=None):
    rclpy.init(args=args)
    node = LiDARTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

