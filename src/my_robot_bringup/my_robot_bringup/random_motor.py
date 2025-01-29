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

def interval_random(v_steps, w_steps):

    v_list = np.linspace(v_min, v_max, v_steps)
    print(v_list)
    
    
class randomMotor(Node):
    def __init__(self, speed, type, v_steps, w_steps):
        super().__init__('random_motor_node')

        self.get_logger().info('Random motor node has started')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = speed#isnt this convolted? 
        self.random_type = type
        self.timer = self.create_timer(self.timer_period, self.publish_random_vel)
        self.v_steps = v_steps
        self.w_steps = w_steps
        self.v_list= np.linspace(v_min, v_max, self.v_steps)
        self.w_list = np.linspace(w_min, w_max, self.w_steps)
  
       
        
    def publish_random_vel(self):

        msg = Twist()
        if self.random_type == "interval":
            print("interval spacking")
            v_val = self.v_list[random.randint(0, self.v_steps - 1)]
            w_val = self.w_list[random.randint(0, self.w_steps - 1)]
            msg.linear.x = v_val 
            msg.linear.z = w_val
            print("v_val", v_val, "w_val", w_val)
        elif self.random_type == "gau":
            msg.linear.x = .1
            msg.angular.z = .1

        self.publisher_.publish(msg)

        self.get_logger().info('Publishing motor movement')


def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=2, help="--interval, publishing freq")
    parser.add_argument("--type", type=str, default='interval', help="type of movement")
    parser.add_argument("--v_steps", type= int, default=8, help="intervals for v")
    parser.add_argument("--w_steps", type= int, default=10, help="intervals for w")
    parsed_args, unknown = parser.parse_known_args()

# Initialize the rclpy library
    rclpy.init(args=args)
# Create and spin the node
    random_motor_node = randomMotor(parsed_args.speed, parsed_args.type, parsed_args.v_steps, parsed_args.w_steps)
    rclpy.spin(random_motor_node)

# Shutdown the node when exiting
    random_motor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
