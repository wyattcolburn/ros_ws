import argparse
import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Odometry 
import math
from scipy.spatial.transform import Rotation as R
class OdomSpoofed(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
                Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Odometry,
                '/odom_local', 10)
        self.forward_offset = .25
    def odom_callback(self, msg):
        new_msg = Odometry() #odom type message
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id

        self.get_logger().info(f"subscriber to odom:")

        
        # Extract orientation quaternion and convert to yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = R.from_quat([qx, qy, qz, qw]).as_euler('xyz')[2]
        
        # Modify position (example: scale x, shift y)
        new_msg.pose.pose.position.x = msg.pose.pose.position.x * (1 + self.forward_offset*math.sin(yaw)) 
        new_msg.pose.pose.position.y = msg.pose.pose.position.y * (1 + self.forward_offset*math.cos(yaw))
        new_msg.pose.pose.position.z = msg.pose.pose.position.z  # Keep Z the same
        

        # Modify orientation (example: rotate yaw by 10 degrees)
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        new_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x 
        new_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y 
        new_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z 
        new_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w 

        new_msg.twist = msg.twist

        self.publisher.publish(new_msg)

def main():
    rclpy.init()
    node = OdomSpoofed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

