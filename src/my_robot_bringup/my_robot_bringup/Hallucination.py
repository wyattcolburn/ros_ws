import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import csv
threshold = 10
class HallucinateNode(Node):
    def __init__(self):
        super().__init__("hallucinate_lidar")

        self.scan_subscriber = self.create_subscription(
                LaserScan, '/scan', self.scan_callback, 10)
        
        self.odom_subscriber = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)

        self.scan_publisher = self.create_publisher(LaserScan, 
                '/scan_spoofed', 10)


        self.robot_x = 0
        self.robot_y = 0
        self.robot_yaw = 0
        self.odom_counter = 0
        self.lidar_data = self.load_lidar_data("/home/wyattcolburn/ros_ws/utils/hallucinated_lidar.csv")
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
    def scan_callback(self, msg):
        
        pass

    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

         # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)

        self.get_logger().info(f"Odom Update - X: {self.robot_x}, Y: {self.robot_y}, Yaw: {self.robot_yaw:.2f} rad")

        self.publish_fake_scan()
    def publish_fake_scan(self):
        """Publish a fake LiDAR scan when scan_callback is never triggered."""
        
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()  # FIXED: Add timestamp
        msg.header.frame_id = "odom"  # Ensure this frame exists in your TF tree


         # Ensure the counter is within range
        if self.odom_counter >= len(self.lidar_data):
            self.get_logger().warn("Reached end of LiDAR CSV data, looping back to start.")
            self.odom_counter = 0  # Reset to the beginning

        # Get the current row from CSV
        spoofed_ranges = self.lidar_data[self.odom_counter]  # Read row from CSV


        # Fake LiDAR scan data
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / len(spoofed_ranges)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = spoofed_ranges

        self.get_logger().info("Publishing default spoofed LiDAR scan...")
        self.scan_publisher.publish(msg)


    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quaternion to yaw angle (in radians)."""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        return math.atan2(siny_cosp, cosy_cosp)
def main():
    rclpy.init()
    node = HallucinateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
