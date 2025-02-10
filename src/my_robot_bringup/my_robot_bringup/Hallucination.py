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
        
#        ranges = msg.ranges
#        self.get_logger().info(f"Num of Lidar Measurements: {len(ranges)}")
#        print("angle min: ", msg.angle_min, "angle max: ", msg.angle_max)
#
#        spoofed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3521030718653303, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35028877800692454, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35033849826926233, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3516695148226535, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3539924247873119, 0.0, 0.0, 0.0, 0.0, 0.35624265667962574, 0.0, 0.0, 0.0, 0.0, 0.35896916910872945, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3642782859128537, 0.0, 0.0, 0.366555526743991, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3742601396140089, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3885746622045802, 0.0, 0.0, 0.0, 0.39311904498060996, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4116316209429074, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4195786434120114, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.43054814705768896, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4447237669429791, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5105549872458599, 0.5119934301855501, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.528985075957411, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5426764814467768, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5532535964138672, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5621960141935257, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5708177185477907, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5976065805504778, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6127738631954845, 0.0, 0.0, 0.0, 0.0, 0.6170305783039631, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.630978937330418, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6363062483087667, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.640781372059174, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6457073811610331, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6487822529677084, 0.0, 0.0, 0.0, 0.6493518932268147, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6488731899647635, 0.0, 0.0, 0.0, 0.6481509180751465, 0.0, 0.0, 0.0, 0.6472515027817098, 0.0, 0.0, 0.0, 0.6461755988454269, 0.0, 0.0, 0.0, 0.6449239917568906, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6306989536543167, 0.0, 0.0, 0.0, 0.6280451145540976, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6149855269603547, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6031400929337459, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5887108103946477, 0.0, 0.0, 0.0, 0.0, 0.5831646263470671, 0.0, 0.0, 0.0, 0.0, 0.5774237838858755, 0.0, 0.0, 0.0, 0.0, 0.5714965039505581, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5578437751402753, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5407394690765467, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5170930821218288, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.49977296398464116, 0.49831339460741014, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4556854969014506, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.43980360759789405, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.42855019834976477, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4136759559796221, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.39377329169073244, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3678013073570486, 0.0, 0.0, 0.0, 0.0, 0.36395932216256144, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3555465027622983, 0.0, 0.0]
#        spoofed_ranges = spoofed
#        spoofed_scan = LaserScan() # type of topic
#        spoofed_scan.header = msg.header
#        spoofed_scan.angle_min = msg.angle_min
#        spoofed_scan.angle_max = msg.angle_max
#        spoofed_scan.angle_increment = msg.angle_increment
#        spoofed_scan.time_increment = msg.time_increment
#        spoofed_scan.scan_time = msg.scan_time
#        spoofed_scan.range_min = msg.range_min
#        spoofed_scan.range_max = msg.range_max 
#        spoofed_scan.ranges = spoofed_ranges  # Use the modified ranges
#
#        self.get_logger().info(f"num of spoofed measurements {len(spoofed_ranges)}")
#        # Publish the spoofed LiDAR data to the fake topic
#        self.scan_publisher.publish(spoofed_scan)
#        self.get_logger().info("Published spoofed LiDAR data to /scan_spoofed")
#
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

        # Fake LiDAR scan data
        spoofed = [1.5] * 640  # Example: Set all range values to 1.5m
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / len(spoofed)
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0
        msg.ranges = spoofed

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
