import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time
import argparse
import numpy as np
from scipy.stats import truncnorm
import pandas as pd

class TestCMD(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('predictions output')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #get this from training.launch.py
        csv_file = '/home/wyattcolburn/model/test1/output_data/cmd_vel.csv'
        
        self.cmd_data = pd.read_csv(csv_file)
        self.index = 0
        self.max_index = len(self.cmd_data)
        print(f"Loaded {self.max_index} commands")
        
        # Start a timer that will call publish() every 0.2 seconds
        self.timer = self.create_timer(0.05, self.publish)
    
    def publish(self):
        if self.index < self.max_index:
            msg = Twist()
            values = self.cmd_data.iloc[self.index]
            msg.linear.x = values[0]
            msg.angular.z = values[1]
            self.publisher_.publish(msg)
            
            self.get_logger().info(f'Published command {self.index+1}/{self.max_index}')
            self.index += 1
        else:
            self.get_logger().info('Published all motor movements, stopping timer')
            self.timer.cancel()  # Stop the timer once all commands are published

def main(args=None):
    rclpy.init(args=args)
    test_node = TestCMD()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
