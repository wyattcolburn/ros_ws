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

        angle_increment = 360 / num_readings
        
        
        height = .5
        width = .2

        target_theta_height = math.atan(height/width)
        target_theta_width = math.atan(width/height)


        scans_per_half_height = math.degrees(target_theta_height) / angle_increment
        print(f"scans per half height {scans_per_half_height}")
        scans_per_half_width = math.degrees(target_theta_width) / angle_increment 
        total_scans = 2*scans_per_half_height + 2 * scans_per_half_width

        corner_list = [0]
        for i in range(8): 
            if i in (1,2,5,6):
                corner_list.append(corner_list[-1]+scans_per_half_width)
            else:
                corner_list.append(corner_list[-1] + scans_per_half_height)
            print(corner_list)
        dimensions = [width, height, height, width, width, height, height, width]
        #for i in range(5):
        #    for j in range(int(corner_list[i+1] -corner_list[i])):
        #        print(f"**********************{i}")
        #        print(corner_list[i])
        #        print(int(corner_list[i+1] - corner_list[i]));
        #        theta_degrees = j * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = height / math.cos(theta_rad + 1E-5)
        #        spoofed_ranges[j] = hyp
        print(f"length of corner_list {len(corner_list)}")
        for corner_list_counter in range(len(corner_list)-1):
            print("entering loop")
            start_index = corner_list[corner_list_counter]
            end_index = corner_list[corner_list_counter + 1]
            dim = dimensions[corner_list_counter]
            print(f"start index {start_index} end index {end_index} dim {dim}") 
            for pixel_counter in range(int(start_index), int(end_index)):
                if corner_list_counter % 2 == 0:
                    theta_degrees = (pixel_counter - start_index) * .5625
                else:
                    theta_degrees = (end_index - pixel_counter) * .5625
                theta_rad = math.radians(theta_degrees)
                hyp = dim/math.cos(theta_rad + 1E-5)
                spoofed_ranges[pixel_counter] = hyp
        #for i in range(num_readings):


        #    # right side top    
        #    if (0 <= i) and (112 >= i):
        #        theta_degrees = (i) * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = width/ (math.cos(theta_rad + 1E-5))
        #        print(f"hyp : {hyp} i : {i} theta_degrees {theta_degrees}")
        #        spoofed_ranges[i] = hyp
        #   #top right side
        #    elif (112 <= i) and (160 >= i):
        #        theta_degrees = (160-i) * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = height / (math.cos(theta_rad + 1E-5))
        #        
        #        spoofed_ranges[i] = hyp
        ##    #top left side    
        #    elif (160 <= i) and (207>= i):
        #        theta_degrees = (i -160 ) * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = height / (math.cos(theta_rad)) + 1E-5
        #        spoofed_ranges[i] = hyp
        ##    #left top side        
        #    elif (207 <= i) and (320 >= i):
        #        theta_degrees = (320-i ) * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = width/ (math.cos(theta_rad + 1E-5))
        #        spoofed_ranges[i] = hyp
        ##    #left bottom side
        #    elif (320 <= i) and (432 >= i):
        #        theta_degrees = (i -320 ) * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = width / (math.cos(theta_rad + 1E-5))
        #        spoofed_ranges[i] = hyp
        ##    #bottom left side
        #    elif (i >= 432) and (i <= 480):

        #        theta_degrees = (480 - i)* .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = height / math.cos(theta_rad + 1E-5)
        #        spoofed_ranges[i] = hyp
        ##    #bottom right side
        #    elif (480 <= i) and (527 >= i):
        #        theta_degrees = (480-i)* .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = height / (math.cos(theta_rad + 1E-5))
        #        spoofed_ranges[i] = hyp
        ##    #right bottom side
        #    elif (527 <= i) and (640 >= i):
        #        theta_degrees = (640-i ) * .5625
        #        theta_rad = math.radians(theta_degrees)
        #        hyp = width / (math.cos(theta_rad + 1E-5))
        #        spoofed_ranges[i] = hyp


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
