"""
This is the node which is used to create the random walks that facilites training
There are things to investigate here: how random is created, what kind of random?
                                                   
"""


import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import time
import argparse
import numpy as np
from scipy.stats import truncnorm
v_min = 0
v_max = .4

w_min = -1.4
w_max = 1.4

def truncated_gaussian(min_val, max_val, mu, sigma):
    #mu is the mean?, sigma is the spread of the numbers from the mean
    a, b = (min_val - mu) / sigma, (max_val - mu) / sigma  # Normalized range
    return truncnorm.rvs(a, b, loc=mu, scale=sigma)  # Sample from truncated distribution

def prev_based_gaussian(min_val, max_val, prev_mu, sigma):

    a, b = (min_val - prev_mu) / sigma, (max_val - prev_mu) / sigma  # normalized range
    return truncnorm.rvs(a, b, loc=prev_mu, scale=sigma)  # sample from truncated distribution

    
class randomMotor(Node):
    def __init__(self):
        super().__init__('random_motor_node')

        self.get_logger().info('Random motor node has started **************************************************************')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = .05#isnt this convolted? 
        self.random_type = type
        self.timer = self.create_timer(self.timer_period, self.publish_random_vel)
        self.start_mu = 0
        self.prev_mu = 0
        self.current_mu = 0
        
    def publish_random_vel(self):


        ## .2 with 10

        msg = Twist()
        msg.linear.x = truncated_gaussian(0, .4, .2, .08)
        new_angular = prev_based_gaussian(-1.4, 1.4, self.prev_mu, .1)
        msg.angular.z = new_angular
        self.prev_mu = new_angular
        self.publisher_.publish(msg)

        self.get_logger().info('Publishing motor movement')


def main(args=None):

# Initialize the rclpy library
    rclpy.init()
# Create and spin the node
    random_motor_node = randomMotor()
    rclpy.spin(random_motor_node)

# Shutdown the node when exiting
    random_motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
