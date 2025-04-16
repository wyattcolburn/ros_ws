import csv

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time
import argparse
import numpy as np
from scipy.stats import truncnorm
import pandas as pd

from sensor_msgs.msg import LaserScan
# To do first make sure this can move the robot according to the random motor movements from combined_dir, bang!! 

# See raw lidar

# Convert it to map frame


class validate(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('predictions output')
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10) 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.hall_pub_ = self.create_publisher(LaserScan, '/HallScan', 10)
        
        #get this from training.launch.py
        csv_file = '/home/wyattcolburn/model/training/train_1/cmd_vel_output.csv'
        
        self.cmd_data = pd.read_csv(csv_file)
        self.index = 0
        self.max_index = len(self.cmd_data)
        print(f"Loaded {self.max_index} commands")
        
        # Start a timer that will call publish() every 0.2 seconds
        self.timer = self.create_timer(0.05, self.publish)
        self.lidar_data = self.load_lidar_data('/home/wyattcolburn/model/training/train_1/lidar_data.csv')

        print(f"size of cmd data : {len(self.cmd_data)} and lidar data size : {len(self.lidar_data)}")
    
    def publish(self):
        if self.index < self.max_index:
            msg = Twist()
            values = self.cmd_data.iloc[self.index]
            msg.linear.x = values[1]
            msg.angular.z = values[2]
            self.publisher_.publish(msg)
            
            self.get_logger().info(f'Published command {self.index+1}/{self.max_index}')
            self.index += 1
        else:
            self.get_logger().info('Published all motor movements, stopping timer')
            self.timer.cancel()  # Stop the timer once all commands are published

    def scan_callback(self, msg):
        spoofed_scan = LaserScan()
        spoofed_scan.header = msg.header
        spoofed_scan.angle_min = msg.angle_min
        spoofed_scan.angle_max = msg.angle_max
        spoofed_scan.angle_increment = msg.angle_increment
        spoofed_scan.time_increment = msg.time_increment
        spoofed_scan.scan_time = msg.scan_time
        spoofed_scan.range_min = msg.range_min
        spoofed_scan.range_max = msg.range_max 
        spoofed_scan.ranges= self.lidar_data[self.index]  # Read row from CSV
        self.hall_pub_.publish(spoofed_scan)


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


def main(args=None):
    rclpy.init(args=args)
    test_node = validate()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
