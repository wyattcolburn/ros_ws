import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
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

    def scan_callback(self, msg):
        
        ranges = msg.ranges
        self.get_logger().info(f"Num of Lidar Measurements: {len(ranges)}")
        print("angle min: ", msg.angle_min, "angle max: ", msg.angle_max)

        spoofed_ranges =[0.0] * 640
        num_readings = len(spoofed_ranges)  # Total LIDAR readings
        mid_index = num_readings // 2  # Forward direction index

        
        spoofed_ranges[0:20] = [2.0] * 20
        for i in range(num_readings):


            # right side top    
            if (0 <= i) and (80 >= i):
                theta_degrees = (i) * .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / (math.cos(theta_rad + 1E-5))
                spoofed_ranges[i] = hyp
            #top right side
            elif (80 <= i) and (160 >= i):
                theta_degrees = (160-i) * .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / (math.cos(theta_rad + 1E-5))
                spoofed_ranges[i] = hyp
            #top left side    
            elif (160 <= i) and (240 >= i):
                theta_degrees = (i -160 ) * .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / (math.cos(theta_rad)) + 1E-5
                spoofed_ranges[i] = hyp
            #left top side        
            elif (240 <= i) and (320 >= i):
                theta_degrees = (i -320 ) * .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / (math.cos(theta_rad + 1E-5))
                spoofed_ranges[i] = hyp
            #left bottom side
            elif (320 <= i) and (400 >= i):
                theta_degrees = (i -320 ) * .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / (math.cos(theta_rad + 1E-5))
                spoofed_ranges[i] = hyp
            #bottom left side
            elif (i >= 400) and (i <= 480):

                theta_degrees = (480 - i)* .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / math.cos(theta_rad + 1E-5)
                spoofed_ranges[i] = hyp
            #bottom right side
            elif (480 <= i) and (560 >= i):
                theta_degrees = (480-i)* .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / (math.cos(theta_rad + 1E-5))
                spoofed_ranges[i] = hyp
            #right bottom side
            elif (560 <= i) and (640 >= i):
                theta_degrees = (640-i ) * .5625
                theta_rad = math.radians(theta_degrees)
                hyp = .5 / (math.cos(theta_rad + 1E-5))
                spoofed_ranges[i] = hyp

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

        self.get_logger().info(f"num of spoofed measurements {len(spoofed_ranges)}")
        # Publish the spoofed LiDAR data to the fake topic
        self.scan_publisher.publish(spoofed_scan)
        self.get_logger().info("Published spoofed LiDAR data to /scan_spoofed")



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
