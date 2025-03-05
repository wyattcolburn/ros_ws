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
        
        self.lidar_data = self.load_lidar_data("/home/wyattcolburn/ros_ws/utils/carlos_lidar_0.csv")
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
        
        self.get_logger().info(f"publishing faked scans with scan_callback")
        
        spoofed_scan = LaserScan()
        spoofed_scan.header = msg.header
        spoofed_scan.angle_min = msg.angle_min
        spoofed_scan.angle_max = msg.angle_max
        spoofed_scan.angle_increment = msg.angle_increment
        spoofed_scan.time_increment = msg.time_increment
        spoofed_scan.scan_time = msg.scan_time
        spoofed_scan.range_min = msg.range_min
        spoofed_scan.range_max = msg.range_max 
        spoofed_scan.ranges= self.lidar_data[self.odom_counter]  # Read row from CSV
        self.odom_counter +=1
        if self.odom_counter > len(self.lidar_data) -1:
            self.odom_counter = 0
            self.get_logger().info(f"starting over lidar data")
    
        self.scan_publisher.publish(spoofed_scan)
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
