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
        self.lidar_data = self.load_lidar_data("/home/wyattcolburn/ros_ws/utils/output_perp.csv")
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

        #ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom turtlebot4/rplidar_link/rplidar
        #creates the correct frame
        pass 
        ranges = msg.ranges
        self.get_logger().info(f"Num of Lidar Measurements: {len(ranges)}")
        self.get_logger().info(f"header info {msg.header}")
        print("angle min: ", msg.angle_min, "angle max: ", msg.angle_max)
        
        
        spoofed_ranges = self.lidar_data[self.odom_counter]  # Read row from CSV
        #spoofed_ranges = [1.5] * 640
        spoofed_scan = LaserScan() # type of topic
        spoofed_scan.header = msg.header
        spoofed_scan.angle_min = msg.angle_min
        spoofed_scan.angle_max = msg.angle_max
        spoofed_scan.angle_increment = msg.angle_increment
        spoofed_scan.time_increment = msg.time_increment
        spoofed_scan.scan_time = msg.scan_time
        spoofed_scan.range_min = msg.range_min
        spoofed_scan.range_max = msg.range_max 
        spoofed_scan.ranges = spoofed_ranges  # Use the modified ranges

        self.get_logger().info(f"num of spoofed measurements {len(spoofed_ranges)} odom_counter : {self.odom_counter}")
        # Publish the spoofed LiDAR data to the fake topic
        self.scan_publisher.publish(spoofed_scan)
        self.get_logger().info("Published spoofed LiDAR data to /scan_spoofed")
        
        #height = 3.0
        #width = 1.0
        #max_range = 12.0
        #target_theta_height = math.atan(height/width)
        #target_theta_width = math.atan(width/height)


        #scans_per_half_height = round(math.degrees(target_theta_height) / angle_increment)
        #print(f"scans per half height {scans_per_half_height}")
        #scans_per_half_width = round(math.degrees(target_theta_width) / angle_increment)
        #total_scans = 2*scans_per_half_height + 2 * scans_per_half_width

        #corner_list = [0]
        #for i in range(8): 
        #    if i in (1,2,5,6):
        #        corner_list.append(corner_list[-1]+scans_per_half_width)
        #    else:
        #        corner_list.append(corner_list[-1] + scans_per_half_height)
        #    print(corner_list)
        #dimensions = [width, height, height, width, width, height, height, width]
        #
        #print(f"length of corner_list {len(corner_list)}")
        #for corner_list_counter in range(len(corner_list)-1):
        #    print("entering loop")
        #    start_index = corner_list[corner_list_counter]
        #    end_index = corner_list[corner_list_counter + 1]
        #    dim = dimensions[corner_list_counter]
        #    print(f"start index {start_index} end index {end_index} dim {dim}") 
        #    for pixel_counter in range(int(start_index), int(end_index)):
        #        if corner_list_counter % 2 == 0:
        #            theta_degrees = (pixel_counter - start_index) * .5625
        #        else:
        #            theta_degrees = (end_index - pixel_counter) * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = dim/math.cos(theta_rad + 1E-5)
        #        spoofed_ranges[pixel_counter] = hyp
        #




    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

         # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = self.quaternion_to_yaw(qx, qy, qz, qw)
        self.odom_counter += 1
        self.get_logger().info(f"Odom Update - X: {self.robot_x}, Y: {self.robot_y}, Yaw: {self.robot_yaw:.2f} rad")

        #self.publish_fake_scan()
    def publish_fake_scan(self):
        """Publish a fake LiDAR scan when scan_callback is never triggered."""
        
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()  # FIXED: Add timestamp
        msg.header.frame_id = msg.header.frame_id = "turtlebot4/rplidar_link/rplidar')" #"base_link"
# Ensure this frame exists in your TF tree


         # Ensure the counter is within range
        if self.odom_counter >= len(self.lidar_data):
        
            self.get_logger().warn("Reached end of LiDAR CSV data, looping back to start.")
            self.odom_counter = 0  # Reset to the beginning
        self.get_logger().info(f"odom counter {self.odom_counter}")
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

            # Transform LiDAR points based on robot position
        transformed_ranges = []
        for i, r in enumerate(spoofed_ranges):
            if r <= 0.1 or r > 10.0:
                transformed_ranges.append(float('inf'))  # Ignore invalid points
                continue

            angle = msg.angle_min + i * msg.angle_increment
            # Transform each scan point by robot position & yaw
            x = r * math.cos(angle) + self.robot_x
            y = r * math.sin(angle) + self.robot_y

            # Convert back to LiDAR polar coordinates relative to base_link
            transformed_r = math.sqrt((x - self.robot_x) ** 2 + (y - self.robot_y) ** 2)
            transformed_ranges.append(transformed_r)
        msg.ranges = transformed_ranges 

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
