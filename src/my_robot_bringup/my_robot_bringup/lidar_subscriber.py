import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LiDARSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription

        self.lidar_data = self.load_lidar_data("/home/wyattcolburn/ros_ws/utils/carlos_lidar_0.csv")
        self.label_data = None

    def load_label_data(self, csv_file):
        return
    def load_lidar_data(self, csv_file):
        """Loads LiDAR data from a CSV file and returns it as a list of lists."""
        lidar_values = []
        try:
            with open(csv_file, "r") as file:
                reader = csv.reader(file)
                for row in reader:
                    lidar_values.append([float(value) for value in row])  # Convert to float
            self.get_logger().info(f"Loaded {len(lidar_values)} LiDAR scans from {csv_file}")
        except Exception as e:
            self.get_logger().error(f"Error loading CSV: {e}")
        return lidar_values
    def lidar_callback(self, msg):
        non_zero_ranges = [value for value in msg.ranges if value > 0.0]
        if non_zero_ranges:
            self.get_logger().info(f"Non-zero LiDAR ranges: {non_zero_ranges}")
        else:
            self.get_logger().info("No non-zero ranges found.")

def main(args=None):
    rclpy.init(args=args)
    node = LiDARSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

