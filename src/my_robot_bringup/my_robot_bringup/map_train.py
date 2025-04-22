import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # <-- this registers the PointStamped type with TF2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import matplotlib.pyplot as plt

import pandas as pd
def odom_to_map(node, odom_x, odom_y, odom_frame='odom', map_frame='map'):
    """
    Convert coordinates from odometry frame to map frame using ROS2 TF2.
    
    Parameters:
    - node: ROS2 node instance
    - odom_x: x coordinate in odometry frame
    - odom_y: y coordinate in odometry frame
    - odom_frame: name of the odometry frame (default: 'odom')
    - map_frame: name of the map frame (default: 'map')
    
    Returns:
    - (map_x, map_y): coordinates in map frame
    - None if transformation failed
    """
    # Create point in odometry frame
    point = PointStamped()
    point.header.stamp = rclpy.time.Time().to_msg()

    point.header.frame_id = odom_frame
    point.point.x = float(odom_x)
    point.point.y = float(odom_y)
    point.point.z = 0.0
    
    try:
        # Transform point from odom frame to map frame
        transformed_point = node.tf_buffer.transform(point, map_frame, rclpy.duration.Duration(seconds=0.5))
        return transformed_point.point.x, transformed_point.point.y
    except TransformException as ex:
        node.get_logger().warn(f"Could not transform point from {odom_frame} to {map_frame}: {ex}")
        return None

class MapTraining(Node):
    def __init__(self):
        super().__init__('map_training_node')

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data setup
        self.odom_file = "/home/wyattcolburn/model/test1/input_data/odom_data.csv"
        self.odom_x = None
        self.odom_y = None
        self.map_x = []
        self.map_y = []
        self.map_points = []
        self.global_path = None
    
        self.dist_between_goals = .2
        # Delay the odom-to-map conversion until TF is ready
        self.odom_timer = self.create_timer(2.0, self.check_tf_and_run)

        # Still keep the live timer for example/demo

    def check_tf_and_run(self):
        # Try checking for transform once TF listener has had time to populate
        if self.tf_buffer.can_transform(
            'map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
        ):
            self.get_logger().info("TF transform map → odom is now available. Proceeding with data transformation.")
            self.odom_timer.cancel()  # Stop calling this timer
            self.odom_data()  # Run the conversion
        else:
            self.get_logger().warn("TF not ready: waiting for map → odom...")

    def odom_data(self):
        df = pd.read_csv(self.odom_file)
        self.odom_x = df['odom_x'].tolist()
        self.odom_y = df['odom_y'].tolist()

        for i, (x_odom, y_odom) in enumerate(zip(self.odom_x, self.odom_y)):
            result = odom_to_map(self, x_odom, y_odom)
            if result is not None:
                x_map, y_map = result
                self.map_x.append(x_map)
                self.map_y.append(y_map)
            else:
                self.get_logger().warn(f"Skipping index {i} due to transform failure.")
                self.map_x.append(None)
                self.map_y.append(None)

        self.get_logger().info(f"Transformed {len(self.map_x)} points to map frame.")


        self.map_points = list(zip(self.map_x, self.map_y))
        self.global_path = self.create_path_from_points(self.map_points)
        self.generate_local_goals()
    def timer_callback(self):
        # Live demo: transform one point
        odom_coords = (2.5, 1.7)
        map_coords = odom_to_map(self, odom_coords[0], odom_coords[1])
        if map_coords:
            self.get_logger().info(f"Odometry {odom_coords} → Map {map_coords}")

    def create_path_from_points(self, map_points, frame_id='map'):
        """
        Create a Path message from a list of (x, y) map points.

        Parameters:
        - map_points: List of (x, y) coordinates in the map frame
        - frame_id: The frame ID for the path (default: 'map')

        Returns:
        - path: nav_msgs/Path message
        """
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(map_points):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # If not the last point, set orientation toward the next point
            if i < len(map_points) - 1:
                next_x, next_y = map_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                # Convert yaw to quaternion (simplified)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # For the last point, use the same orientation as the previous one
                pose.pose.orientation.w = 1.0

            path.poses.append(pose)
        print("Path has been created")
        return path

    def distance_between_points(self, point1, point2):
        """
        Calculate Euclidean distance between two points (x1, y1) and (x2, y2)
        """
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def distance_between_poses(self, pose1, pose2):
        """
        Calculate distance between two PoseStamped messages
        """
        return self.distance_between_points(
            (pose1.pose.position.x, pose1.pose.position.y),
            (pose2.pose.position.x, pose2.pose.position.y)
        )

    def generate_local_goals(self):
        """
        Generate local goals along the global path based on the current robot position
        """
        if self.global_path is None or len(self.global_path.poses) == 0:
            self.get_logger().warn("Cannot generate local goals: No global path available")
            return

        self.local_goals = []

        # Find the closest point on the path to the current robot position

        # Start from the closest point and generate local goals along the path

        
        accumulated_distance = 0.0
        threshold_distance = .1

        for i in range(len(self.global_path.poses)-1):
            current_pose = self.global_path.poses[i]
            next_pose = self.global_path.poses[i+1]

            # Calculate distance between consecutive points
            segment_distance = self.distance_between_poses(current_pose, next_pose)
            print(segment_distance)
            accumulated_distance +=segment_distance 
            # Add points at regular intervals
            if accumulated_distance >= threshold_distance:
                print("Add a local goal")
                self.local_goals.append(current_pose)
                accumulated_distance = 0
                

        self.get_logger().info(f"Generated {len(self.local_goals)} local goals")
        self.visual_path_goals() 
    def visual_path_goals(self):

        """Takes self.global_path and self.local_goals and plots them
        """

        if self.global_path == None:
            return
        plt.figure(figsize=(8,6))

        for val in self.global_path.poses:
            plt.plot(val.pose.position.x, val.pose.position.y, marker='o', linestyle='-',markersize=3,color='blue')

        for goal in self.local_goals:
            plt.plot(goal.pose.position.x, goal.pose.position.y, marker='o', linestyle='-',markersize=5,color='red')
        plt.show()

    def local_goals(self):

        pass

    def update_local_goals(self):
        pass

    def obstacle_creation(self):
        pass

    def update_obstacles(self):
        
        pass

    def ray_tracining(self):

        pass







def main(args=None):
    rclpy.init(args=args)
    test_node = MapTraining()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
