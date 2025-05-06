"""
How does this run,
We will run test node, which will take the raw motor cmds and publish them so that the robot moves with randomness,
We need to already know how the robot runs to generate the plan, to create the local goals, etc. But to validate we
want to be able to run, so similar to what we are already doing we create CSV files with odom data, could do the same with obstacle data?


Lets say we are odom_rad (x,y), we need to know yaw, and locations of revalent obstacles,  

April 29: Currently, this takes odom_csv, and creates the local goals and obstacles to generate ray traces

"""
import matplotlib.patches as patches
import numpy as np
import rosbag2_py
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped, Twist
import tf2_geometry_msgs  # <-- this registers the PointStamped type with TF2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import matplotlib.pyplot as plt
import csv
import os
from nav_msgs.msg import Odometry 
from visualization_msgs.msg import MarkerArray

from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker

from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
import pandas as pd


class Obstacle():
    def __init__(self, center_x=None, center_y=None, radius=None):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius

class Obstacle_Manager():
    def __init__(self, OFFSET, RADIUS,local_goal_manager= None):
        self.obstacle_array = []
        self.local_goal_manager_ = local_goal_manager
        self.OFFSET = OFFSET    
        self.RADIUS = RADIUS
        self.prev_dir_x = 0
        self.prev_dir_y = 0
    def preprocess_obstacles_with_sampling(self, path, sampling_rate=5):
        """Preprocess obstacles considering sampled path segments for efficiency."""
        obstacle_distances = []
        
        for obs in self.obstacle_array:
            # Calculate minimum distance to sampled path segments
            min_dist_to_any_segment = float('inf')
            closest_segment_index = -1
            
            # Sample segments at regular intervals for efficiency
            for i in range(0, len(path.poses) - 1, sampling_rate):
                p1 = (path.poses[i].pose.position.x, path.poses[i].pose.position.y)
                p2 = (path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y)
                
                dist = self.point_to_line_distance(
                    obs.center_x, obs.center_y, 
                    p1[0], p1[1], p2[0], p2[1]
                ) - obs.radius
                
                if dist < min_dist_to_any_segment:
                    min_dist_to_any_segment = dist
                    closest_segment_index = i
            
            # Store obstacle with its distance data
            obstacle_distances.append({
                'obstacle': obs,
                'min_distance': min_dist_to_any_segment,
                'segment_index': closest_segment_index
            })
        
        return obstacle_distances
    def path_has_changed(self, new_path):
        """Check if the path has changed significantly since last preprocessing."""
        if not hasattr(self, 'last_processed_path_'):
            self.last_processed_path_ = new_path
            return True
            
        # Simple check: compare number of poses
        if len(new_path.poses) != len(self.last_processed_path_.poses):
            self.last_processed_path_ = new_path
            return True
            
        # Check if end points have changed
        if len(new_path.poses) > 0 and len(self.last_processed_path_.poses) > 0:
            new_end = new_path.poses[-1].pose.position
            old_end = self.last_processed_path_.poses[-1].pose.position
            dist = math.sqrt((new_end.x - old_end.x)**2 + (new_end.y - old_end.y)**2)
            if dist > 0.5:  # If end point moved more than 0.5 meters
                self.last_processed_path_ = new_path
                return True
            
        return False
    def get_active_obstacles_with_sampling(self, path, yaw, sampling_rate=5):
        """Get active obstacles based on sampled path segments."""
        # Preprocess all obstacles if not done yet
        if not hasattr(self, 'obstacle_distances_') or self.path_has_changed(path):
            self.obstacle_distances_ = self.preprocess_obstacles_with_sampling(path, sampling_rate)
        
        # Get current robot position
        robot_pos = self.local_goal_manager_.current_odom
        
        # Find current segment index (sampled)
        current_segment_idx = self.find_current_path_segment_sampled(robot_pos, path, sampling_rate)
        
        # Define a window of segments to consider
        segment_window_start = max(0, current_segment_idx - sampling_rate*2)
        segment_window_end = min(len(path.poses) - 1, current_segment_idx + sampling_rate*10)
        
        # Find obstacles relevant to current window of segments
        obstacles_with_data = []
        for obs_data in self.obstacle_distances_:
            obs = obs_data['obstacle']
            segment_idx = obs_data['segment_index']
            
            # Check if obstacle's segment is within our current window of interest
            if segment_window_start <= segment_idx <= segment_window_end:
                # Calculate distance and angle from robot
                dx = obs.center_x - robot_pos[0]
                dy = obs.center_y - robot_pos[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                # Skip distant obstacles
                if dist > 5.0:
                    continue
                    
                # Calculate angle relative to robot heading
                angle = math.atan2(dy, dx) - yaw
                # Normalize angle
                while angle > math.pi:
                    angle -= 2 * math.pi
                while angle < -math.pi:
                    angle += 2 * math.pi
                    
                # Only consider obstacles in front
                if abs(angle) < math.pi/2:
                    # Include obstacle if it's close enough to path to be relevant
                    # but not directly on the path (create a corridor)
                    if 0.2 < obs_data['min_distance'] < 1.5:  # Creates a corridor
                        obstacles_with_data.append((obs, dist))
        
        # Sort by distance and return closest few
        obstacles_with_data.sort(key=lambda x: x[1])
        return [obs for obs, _ in obstacles_with_data[:5]]

    def find_current_path_segment_sampled(self, robot_pos, path, sampling_rate=5):
        """Find which sampled path segment the robot is currently on."""
        min_dist = float('inf')
        current_segment = 0
        
        # Check sampled segments only
        for i in range(0, len(path.poses) - 1, sampling_rate):
            if i+1 >= len(path.poses):
                break
                
            p1 = (path.poses[i].pose.position.x, path.poses[i].pose.position.y)
            p2 = (path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y)
            
            # Distance from robot to this segment
            dist = self.point_to_line_distance(
                robot_pos[0], robot_pos[1],
                p1[0], p1[1], p2[0], p2[1]
            )
            
            # Check if this is the closest segment
            if dist < min_dist:
                min_dist = dist
                current_segment = i
        
        return current_segment
    def preprocess_obstacles(self, path):
        """Preprocess obstacles to eliminate those on the path."""
        valid_obstacles = []
        
        for obs in self.obstacle_array:
            # Check if obstacle is on the path
            path_dist = self.distance_to_path(obs, path)
            
            # Keep obstacles that are NOT on the path
            if path_dist > 0.2:  # More than 0.8 meters from path
                valid_obstacles.append(obs)
        
        # Store this preprocessed list for later use
        self.valid_obstacles_ = valid_obstacles
        print(f"LEN OF VALID_OBSTACLES {len(self.valid_obstacles_)}")

        return valid_obstacles

    def get_active_obstacles_claude(self, path, yaw):
        """Select obstacles based on distance and direction relative to robot."""
        # Get current robot position and orientation
        robot_pos = self.local_goal_manager_.current_odom
        robot_heading = yaw
        
        # Calculate distance and angle to each obstacle
        obstacles_with_data = []
        for obs in self.valid_obstacles_:
            # Calculate distance
            dx = obs.center_x - robot_pos[0]
            dy = obs.center_y - robot_pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            
            # Calculate angle relative to robot heading
            angle = math.atan2(dy, dx) - robot_heading
            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
                
            # Only consider obstacles in front (within ±90° of heading)
            if abs(angle) < math.pi/2 and dist < 5.0:
                # Check if obstacle is close to path
                path_dist = self.distance_to_path(obs, path)
                # Add obstacles that are CLOSE to the path (not far from it)
                if path_dist < 0.8:  # Less than 0.8 meters from path
                    obstacles_with_data.append((obs, dist))
        
        # Sort by distance and take closest N obstacles
        obstacles_with_data.sort(key=lambda x: x[1])
        return [obs for obs, _ in obstacles_with_data[:10]]  # Limit to 10 closest obstacles
    def distance_to_path(self, obstacle, path):
        """Calculate the minimum distance from an obstacle to the planned path.
        
        Args:
            obstacle: The obstacle to check
            
        Returns:
            float: The minimum distance from the obstacle to the path
        """
        # Get the planned path points
        # Assuming self.path contains the planned path points
        
        # Calculate obstacle radius to account for its size
        obstacle_radius = obstacle.radius
        min_distance = float('inf')
        
        for i in range(len(path.poses) - 1):
            # Get the line segment
            p1 = (path.poses[i].pose.position.x, path.poses[i].pose.position.y)
            p2 = (path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y)
            
            # Calculate the distance from the obstacle center to the line segment
            dist = self.point_to_line_distance(
                obstacle.center_x, obstacle.center_y, 
                p1[0], p1[1], p2[0], p2[1]
            )
            
            # Subtract the obstacle radius to get the true edge distance
            edge_distance = dist - obstacle_radius
            min_distance = min(min_distance, edge_distance)
                
        return min_distance

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        """Calculate the shortest distance from a point to a line segment.
        
        Args:
            px, py: The point coordinates
            x1, y1: The first endpoint of the line segment
            x2, y2: The second endpoint of the line segment
            
        Returns:
            float: The shortest distance from the point to the line segment
        """
        # Calculate the line length squared
        line_length_sq = (x2 - x1)**2 + (y2 - y1)**2
        
        # If the line is a point, return the distance to that point
        if line_length_sq == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        # Calculate projection of point onto line
        t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_length_sq))
        
        # Calculate the closest point on the line segment
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        # Return the distance to the closest point
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)

    def create_all_obstacle(self):

        for i, goal in enumerate(self.local_goal_manager_.data):
            current = self.local_goal_manager_.data[i]
            if i + 1 >= len(self.local_goal_manager_.data):
                return
            next = self.local_goal_manager_.data[i+1]
            self.obstacle_creation(current, next)
        print("All obstacles created")
        return

    def obstacle_creation(self, current_local_goal, next_local_goal):

        mid_x = (current_local_goal.pose.position.x + next_local_goal.pose.position.x) / 2
        mid_y = (current_local_goal.pose.position.y + next_local_goal.pose.position.y) / 2
        dir_x = next_local_goal.pose.position.x - current_local_goal.pose.position.x
        dir_y = next_local_goal.pose.position.y - current_local_goal.pose.position.y
        # Normalize
        length = math.sqrt(dir_x * dir_x + dir_y * dir_y)

        if (length > 0):
            dir_x /= length
            dir_y /= length

        perp_x = -dir_y
        perp_y = dir_x

        offset_x = perp_x * self.OFFSET
        offset_y = perp_y * self.OFFSET

        ob1 = Obstacle()
        ob1.center_x = mid_x + offset_x
        ob1.center_y = mid_y + offset_y
        ob1.radius = self.RADIUS

        ob2 = Obstacle()
        ob2.center_x = mid_x - offset_x
        ob2.center_y = mid_y - offset_y
        ob2.radius = self.RADIUS
       
        self.obstacle_array.append(ob1)
        self.obstacle_array.append(ob2)


class Local_Goal_Manager():
    def __init__(self, current_odom, global_path = None):
        self.current_odom = (None, None)
        self.data = []
        self.global_path = global_path
        self.current_lg_counter = 0
        self.current_odom = current_odom
    def generate_local_goals_claude(self, global_path):
        """
        Modified to create more local goals in areas with high curvature
        """
        if self.global_path is None:
            print("Cannot generate local goals: No global path available")
            self.global_path = global_path
            return

        accumulated_distance = 0.0
        base_threshold = 0.1
        
        for i in range(len(self.global_path.poses)-1):
            current_pose = self.global_path.poses[i]
            next_pose = self.global_path.poses[i+1]
            
            # Calculate direction change if not at beginning
            curvature_factor = 1.0
            if i > 0:
                prev_pose = self.global_path.poses[i-1]
                prev_dir_x = current_pose.pose.position.x - prev_pose.pose.position.x
                prev_dir_y = current_pose.pose.position.y - prev_pose.pose.position.y
                curr_dir_x = next_pose.pose.position.x - current_pose.pose.position.x
                curr_dir_y = next_pose.pose.position.y - current_pose.pose.position.y
                
                # Normalize vectors
                prev_len = math.sqrt(prev_dir_x**2 + prev_dir_y**2)
                curr_len = math.sqrt(curr_dir_x**2 + curr_dir_y**2)
                
                if prev_len > 0 and curr_len > 0:
                    prev_dir_x /= prev_len
                    prev_dir_y /= prev_len
                    curr_dir_x /= curr_len
                    curr_dir_y /= curr_len
                    
                    # Dot product gives cosine of angle between vectors
                    dot_product = prev_dir_x * curr_dir_x + prev_dir_y * curr_dir_y
                    angle = math.acos(max(-1.0, min(1.0, dot_product)))
                    
                    # Adjust threshold based on curvature (smaller threshold = more points)
                    curvature_factor = max(0.3, 1.0 - angle/math.pi)
            
            # Calculate distance with adaptive threshold
            segment_distance = self.distance_between_poses(current_pose, next_pose)
            adaptive_threshold = base_threshold * curvature_factor
            
            accumulated_distance += segment_distance
            if accumulated_distance >= adaptive_threshold:
                self.data.append(current_pose)
                accumulated_distance = 0   
    def generate_local_goals(self, global_path):
        """
        Generate local goals along the global path based on the current robot position
        """
        if self.global_path is None:
            print("Cannot generate local goals: No global path available")
            self.global_path = global_path
            return

        # Find the closest point on the path to the current robot position

        # Start from the closest point and generate local goals along the path

        
        accumulated_distance = 0.0
        threshold_distance = .08

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
                self.data.append(current_pose)
                accumulated_distance = 0
                

        print(f"Generated {len(self.data)} local goals")
        self.current_lg = (self.data[self.current_lg_counter].pose.position.x, self.data[self.current_lg_counter].pose.position.y)

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
    def update_claude(self, current_odom=None):
        if self.global_path is None or current_odom[0] is None:
            return
            
        self.current_odom = current_odom  # Make sure to update this field
        self.current_lg = (self.data[self.current_lg_counter].pose.position.x, self.data[self.current_lg_counter].pose.position.y)
        dist_to_goal = self.distance_between_points(self.current_lg, current_odom)
        
        # Look ahead more dynamically
        look_ahead = 3  # Check several goals ahead
        best_index = self.current_lg_counter
        best_dist = dist_to_goal
        
        # Check if any of the next few local goals are closer
        for i in range(1, look_ahead + 1):
            next_index = self.current_lg_counter + i
            if next_index < len(self.data):
                next_lg = (
                    self.data[next_index].pose.position.x,
                    self.data[next_index].pose.position.y,
                )
                next_dist = self.distance_between_points(next_lg, current_odom)
                
                if next_dist < best_dist:
                    best_dist = next_dist
                    best_index = next_index
        
        # Update to the best local goal found
        if best_index != self.current_lg_counter:
            self.current_lg_counter = best_index
            self.current_lg = (
                self.data[self.current_lg_counter].pose.position.x,
                self.data[self.current_lg_counter].pose.position.y,
            )
        elif dist_to_goal < 0.05:  # If we're close to the current goal
            self.current_lg_counter += 1
            if self.current_lg_counter < len(self.data):
                self.current_lg = (
                    self.data[self.current_lg_counter].pose.position.x,
                    self.data[self.current_lg_counter].pose.position.y,
                )
        
        return self.current_lg_counter
    def update(self, current_odom=None):
        # Comparing poses so might not work
        if self.global_path is None or self.current_lg is None or self.current_odom[0] is None:
            return
        self.current_odom = current_odom # updating every call 
        dist_to_goal = self.distance_between_points(self.current_lg, current_odom)
        
        if dist_to_goal < .05:
            self.current_lg_counter += 1
            if self.current_lg_counter < len(self.data):
                if self.data[self.current_lg_counter].pose.position.x:
                    self.current_lg = (self.data[self.current_lg_counter].pose.position.x, self.data[self.current_lg_counter].pose.position.y)
            print("Have updated current local goal:")
        else:
            # Check if the *next* local goal is even closer
            next_lg_counter = self.current_lg_counter + 1
            if next_lg_counter < len(self.data):
                next_lg = (
                    self.data[next_lg_counter].pose.position.x,
                    self.data[next_lg_counter].pose.position.y,
                )
                dist_to_next = self.distance_between_points(next_lg, current_odom)

                if dist_to_next < dist_to_goal:
                    print(f"Jumping to closer local goal {next_lg_counter} (dist: {dist_to_next:.4f})")
                    self.current_lg_counter = next_lg_counter
                    self.current_lg = next_lg
            print(f"Have yet to reach local goal: dist to lg = {dist_to_goal:.4f}")
            print(f"Currently at local goal index: {self.current_lg_counter}")
            return self.current_lg_counter
    def get_local_goal_counter(self):
        return self.current_lg_counter
    
    def get_coords(self):
        return (self.current_lg[0], self.current_lg[1])

    def set_global_path(self, new_global_path):
        if new_global_path is not None and len(new_global_path.poses) > 0:
            self.global_path = new_global_path
            self.current_lg_counter = 0
            return True
        return False

    def upscale_local_goal(self, start_lg, map_points, output_csv):

        
        currentLocalGoal = (start_lg[0],start_lg[1])
        lg_upsampled = [] 
        lg_yaw_upsampled = []
        lgCounter = 0
        odomCounter = 0
        while len(map_points) != len(lg_upsampled):
            odomPoint = (map_points[odomCounter][0], map_points[odomCounter][1])
            if odomPoint == currentLocalGoal:
                # if have reached the local goal, update to next local goal
                if lgCounter +1 < len(self.data):
                    lgCounter += 1
                    currentLocalGoal = (self.data[lgCounter].pose.position.x, self.data[lgCounter].pose.position.y)
                    print("success, local goal has been reached")

            
            yaw = self.get_yaw(self.data[lgCounter].pose)
            lg_yaw_upsampled.append(yaw)
            lg_upsampled.append(currentLocalGoal)
            odomCounter += 1
            print("len of lg_upsampled", len(lg_upsampled), len(map_points), lgCounter, len(self.data))
        print(len(lg_upsampled) == len(map_points))
        



        with open(output_csv, 'w', newline = '') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['local_goals_x', 'local_goals_y', 'local_goals_yaw'])
        
            for i in range(len(lg_upsampled)):
                writer.writerow([lg_upsampled[i][0], lg_upsampled[i][1], lg_yaw_upsampled[i]])

    def get_yaw(self, pose: Pose) -> float:
        quat = (pose.orientation.x, pose.orientation.y,pose.orientation.z,
                pose.orientation.w)
        _, _, yaw = euler_from_quaternion(quat)
        return yaw
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
        self.odom_x = None
        self.odom_y = None
        self.map_x = []
        self.map_y = []
        self.map_points = []
        self.global_path = None
        
        self.current_odom = (0.0, 0.0)
        
        self.OFFSET = 1.0
        self.RADIUS = .5
        self.NUM_VALID_OBS = 20
        self.NUM_LIDAR = 1080
        self.Obstacle_list = []

        self.Obstacle_list = []
        self.dist_between_goals = .2
        # Delay the odom-to-map conversion until TF is ready
        self.odom_timer = self.create_timer(2.0, self.check_tf_and_run)


        
        self.distances = [0] * self.NUM_LIDAR


        # Files for training data to be stored
        self.input_bag = "/home/wyattcolburn/ros_ws/rosbag2_2025_05_02-15_39_18/"
        self.frame_dkr = f"{self.input_bag}/input_data/"
        os.makedirs(self.frame_dkr, exist_ok=True)
        self.odom_csv_file = os.path.join(self.frame_dkr, "odom_data.csv")
        self.cmd_csv = os.path.join(self.frame_dkr, "cmd_vel.csv")
        self.lidar_file = os.path.join(self.frame_dkr, "lidar_data.csv")
        
        self.local_goals_output = os.path.join(self.frame_dkr, "local_goals.csv")
        self.cmd_output_csv = os.path.join(self.frame_dkr, "cmd_vel_output.csv")
        

        training_output = os.path.join(self.frame_dkr, "big_csv.csv")
        path_output = os.path.join(self.frame_dkr, "odom_path")
        obstacles_output = os.path.join(self.frame_dkr, "obactles.csv")
    def validate_obstacles(self):
        output_folder = "obstacle_validation"
        os.makedirs(output_folder, exist_ok=True)
        
        segment_size = 20  # Number of odom points per segment
        window_size = 8    # Number of local goals to include in each window
        
        # Iterate through the odom path in segments
        for segment_start in range(0, len(self.map_points), segment_size):
            segment_end = min(segment_start + segment_size, len(self.map_points))
            
            # Create a figure for this segment
            plt.figure(figsize=(12, 8))
            
            # Get the current segment of the path
            segment_points = self.map_points[segment_start:segment_end]
            segment_x = [point[0] for point in segment_points]
            segment_y = [point[1] for point in segment_points]
            
            # Plot the entire path (lightly) for context
            path_x = [point[0] for point in self.map_points]
            path_y = [point[1] for point in self.map_points]
            plt.plot(path_x, path_y, 'k-', linewidth=0.5, alpha=0.2, label='Full Path')
            
            # Highlight the current segment
            plt.plot(segment_x, segment_y, 'b-', linewidth=2, label=f'Segment {segment_start//segment_size + 1}')
            
            # Find center point of this segment
            if segment_points:
                segment_center_x = sum(point[0] for point in segment_points) / len(segment_points)
                segment_center_y = sum(point[1] for point in segment_points) / len(segment_points)
                segment_center = (segment_center_x, segment_center_y)
                
                # Find the closest local goal to the segment center
                closest_lg_idx = 0
                min_dist = float('inf')
                
                for i, goal in enumerate(self.local_goal_manager_.data):
                    goal_pos = (goal.pose.position.x, goal.pose.position.y)
                    dist = math.sqrt((goal_pos[0] - segment_center[0])**2 + 
                                     (goal_pos[1] - segment_center[1])**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_lg_idx = i
                
                # Create a window of local goals centered around the closest one
                window_start = max(0, closest_lg_idx - window_size // 2)
                window_end = min(len(self.local_goal_manager_.data), window_start + window_size)
                
                # If we're at the end, adjust the window start to maintain the window size
                if window_end - window_start < window_size:
                    window_start = max(0, window_end - window_size)
                
                window_local_goals = self.local_goal_manager_.data[window_start:window_end]
                
                # Plot the relevant local goals
                lg_x = [goal.pose.position.x for goal in window_local_goals]
                lg_y = [goal.pose.position.y for goal in window_local_goals]
                plt.scatter(lg_x, lg_y, c='blue', s=50, label='Local Goals')
                
                # Plot segments between consecutive local goals
                for i in range(len(window_local_goals) - 1):
                    current = window_local_goals[i]
                    next_goal = window_local_goals[i+1]
                    plt.plot([current.pose.position.x, next_goal.pose.position.x],
                             [current.pose.position.y, next_goal.pose.position.y],
                             'g-', linewidth=2, alpha=0.7)
                
                # Find obstacles associated with these local goals
                relevant_obstacles = []
                for i in range(window_start, window_end - 1):
                    # Get the two obstacles associated with this local goal segment
                    obs_idx_start = i * 2
                    if obs_idx_start + 1 < len(self.obstacle_manager_.obstacle_array):
                        relevant_obstacles.append(self.obstacle_manager_.obstacle_array[obs_idx_start])
                        relevant_obstacles.append(self.obstacle_manager_.obstacle_array[obs_idx_start + 1])
                
                # Plot the relevant obstacles
                for obstacle in relevant_obstacles:
                    # Draw the obstacle
                    circle = plt.Circle((obstacle.center_x, obstacle.center_y), 
                                       radius=obstacle.radius, 
                                       fill=False, 
                                       color='red', 
                                       linewidth=1)
                    plt.gca().add_patch(circle)
                
                # Connect obstacles to their midpoints
                for i in range(window_start, window_end - 1):
                    if i * 2 + 1 < len(self.obstacle_manager_.obstacle_array):
                        current_lg = self.local_goal_manager_.data[i]
                        next_lg = self.local_goal_manager_.data[i+1]
                        
                        # Calculate midpoint of the segment
                        mid_x = (current_lg.pose.position.x + next_lg.pose.position.x) / 2
                        mid_y = (current_lg.pose.position.y + next_lg.pose.position.y) / 2
                        
                        # Draw the midpoint
                        plt.scatter(mid_x, mid_y, c='purple', s=30)
                        
                        # Draw connecting lines to obstacles
                        obstacle1 = self.obstacle_manager_.obstacle_array[i * 2]
                        obstacle2 = self.obstacle_manager_.obstacle_array[i * 2 + 1]
                        
                        plt.plot([mid_x, obstacle1.center_x], [mid_y, obstacle1.center_y], 
                                'm-', linewidth=1, alpha=0.7)
                        plt.plot([mid_x, obstacle2.center_x], [mid_y, obstacle2.center_y], 
                                'm-', linewidth=1, alpha=0.7)
                
                # Label the first and last points of the segment
                plt.scatter(segment_points[0][0], segment_points[0][1], c='green', s=100, label='Segment Start')
                plt.scatter(segment_points[-1][0], segment_points[-1][1], c='red', s=100, label='Segment End')
            
            # Add segment range to the title
            plt.title(f'Obstacle Validation - Odom Points {segment_start} to {segment_end-1}')
            plt.legend()
            plt.grid(True)
            plt.axis('equal')
            
            # Save this segment's plot
            plt.savefig(f"{output_folder}/validation_segment_{segment_start:04d}_{segment_end-1:04d}.png", dpi=300)
            plt.close()
        
        print(f"Saved obstacle validation plots to {output_folder}/")
   
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
        
        self.save_to_csv(self.input_bag, self.odom_csv_file, '/odom') # turned bag into csv, this is in the odom frame
        
        self.save_to_csv(self.input_bag, self.cmd_csv, '/cmd_vel') # turned bag into csv
        self.oversample_cmdVel3(self.odom_csv_file, self.cmd_csv, self.cmd_output_csv)
        

        df = pd.read_csv(self.odom_csv_file)
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

        self.setup()
        self.get_logger().info(f"Transformed {len(self.map_x)} points to map frame.")


    def setup(self):
        #self.test_obs_700()
        self.map_points = list(zip(self.map_x, self.map_y))
        self.global_path = self.create_path_from_points(self.map_points)

        self.current_odom = (self.map_points[0][0], self.map_points[0][1])
        
        
        self.local_goal_manager_ = Local_Goal_Manager(self.current_odom)
        self.local_goal_manager_.global_path = self.global_path
        self.local_goal_manager_.generate_local_goals_claude(self.global_path)
        self.local_goal_manager_.upscale_local_goal((self.local_goal_manager_.data[0].pose.position.x, self.local_goal_manager_.data[0].pose.position.y), self.map_points, self.local_goals_output) 

        self.obstacle_manager_ = Obstacle_Manager(self.OFFSET, self.RADIUS,self.local_goal_manager_)
        self.obstacle_manager_.create_all_obstacle()
        self.obstacle_manager_.preprocess_obstacles_with_sampling(self.global_path)
        #self.validate_obstacles()
        print("VALIDATED ****************************")
        print(f"FIRST POINT IS ********************************8 {self.map_points[0]}")
        
        print("checking if a local goal exists in map_points")
        # Check if a specific local goal exists in map pointsp
        found, matching_point = self.is_pose_in_map_points(self.local_goal_manager_.data[0], self.map_points)
        if found:
            print(f"local goal exists within map points at {matching_point}")
        else:
            print("error local goal not in map points list")

        output_folder = "obstacle_and_path_2"
        os.makedirs(output_folder, exist_ok=True)
        plt.figure(figsize=(8,6))

        plt.clf()  # Clear previous plot
        ax = plt.gca()
        ax.set_aspect('equal')
        # Replot the base elements
        #plt.plot(self.current_odom[0], self.current_odom[1], marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
        active_obstacle = self.obstacle_manager_.get_active_obstacles_with_sampling(self.global_path, self.get_yaw(self.global_path.poses[0].pose))
        for obstacle in self.obstacle_manager_.obstacle_array:
            circle = patches.Circle(
            (obstacle.center_x, obstacle.center_y),
            radius=obstacle.radius,
            fill=False,
            color='red',
            linewidth=1.5,
            linestyle='-'
    )
            ax.add_patch(circle)

        path_x = [point[0] for point in self.map_points]
        path_y = [point[1] for point in self.map_points]

        # Plot the entire path
        plt.plot(path_x, path_y, marker='o', linestyle='-', markersize=3, color='black', label='odom path')

        frame_path = f"{output_folder}/obst_path.png"
        plt.savefig(frame_path)

        print("Saved png of obstacles and path")
        self.main_loop() 
    
    def main_loop(self):

        output_folder = "may_2"
        os.makedirs(output_folder, exist_ok=True)
        plt.figure(figsize=(8, 6))
        for i, map_point in enumerate(self.map_points):

            self.current_odom = (map_point[0], map_point[1])
            #update_count_val = self.local_goal_manager_.update_claude(self.current_odom)

            self.current_odom_index = i
            self.local_goal_manager_.current_odom = self.current_odom
            #closest_local_goal = self.get_closest_local_goal_index(self.local_goal_manager_.data, self.current_odom[0], self.current_odom[1])
            print(f"frame {i}")
            #print(f"value from update : {update_count_val} closest local goal {closest_local_goal}")
            local_data = self.ray_tracing(self.global_path.poses[i].pose) # get ray tracining values, now need to store them correctly with 
            #print(f"calculating ray tracing for position {i} and {self.global_path.poses[i].pose}")
            self.ray_data_append()

            if i % 10 != 0:
                continue
            plt.clf()  # Clear previous plot
            ax = plt.gca()
            ax.set_aspect('equal')
            # Replot the base elements
            plt.plot(self.current_odom[0], self.current_odom[1], marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
            active_obstacle = self.obstacle_manager_.get_active_obstacles_with_sampling(self.global_path, self.get_yaw(self.global_path.poses[i].pose))


            current_local_goal_count = self.local_goal_manager_.get_local_goal_counter()
            total_goals = len(self.local_goal_manager_.data)
            print("local goal count") 
            valid_border_min = max(0, current_local_goal_count - 4)
            valid_border_max = min(total_goals,  current_local_goal_count+ 20)
            
            local_goal_list = self.local_goal_manager_.data[valid_border_min: valid_border_max]
            print(f"len of local goal list {len(local_goal_list)}")
            for local_goal in local_goal_list:
                plt.scatter(local_goal.pose.position.x, local_goal.pose.position.y, s=100, color='purple')  # s=100 for size equivalent to markersize=10
            for obstacle in active_obstacle:
                circle = patches.Circle(
                (obstacle.center_x, obstacle.center_y),
                radius=obstacle.radius,
                fill=False,
                color='red',
                linewidth=1.5,
                linestyle='-'
        )
                ax.add_patch(circle)

            plt.scatter(self.current_odom[0], self.current_odom[1], color='cyan', s=200, label='robot')
            path_x = [point[0] for point in self.map_points]
            path_y = [point[1] for point in self.map_points]

            # Plot the entire path
            plt.plot(path_x, path_y, marker='o', linestyle='-', markersize=3, color='black', label='odom path')
            self.draw_rays_claude_2(self.current_odom[0], self.current_odom[1], local_data)
            # Save the frame
            
            frame_path = f"{output_folder}/frame_{i:03d}.png"
            plt.savefig(frame_path)
    
        plt.close()
        print("done with main loop")


    def draw_rays_claude_2(self, odom_x, odom_y, lidar_readings):
        # Get robot's yaw from the current pose (you need to pass this as a parameter)
        robot_yaw = self.get_yaw(self.global_path.poses[self.current_odom_index].pose)
        
        # Define lidar offset relative to robot (90 degrees = π/2 radians)
        lidar_offset = -math.pi/2
        
        # Draw rays
        for lidar_counter in range(self.NUM_LIDAR):
            # Calculate ray angle in the global frame
            lidar_angle = lidar_counter * (2*np.pi / self.NUM_LIDAR)
            global_angle = robot_yaw + lidar_angle + lidar_offset
            global_angle = self.normalize_angle(global_angle)
            
            distance = lidar_readings[lidar_counter]
            
            # Ensure reasonable distance values
            if distance <= 0.001 or distance > 50.0:  # Likely invalid value
                continue
                
            # Draw a single line from robot to endpoint using the global angle
            projection_x = odom_x + distance * math.cos(global_angle)
            projection_y = odom_y + distance * math.sin(global_angle)
            
            # Individual rays should be single lines, not connected

            plt.plot([odom_x, projection_x], [odom_y, projection_y], 
                 linestyle='-', color='green', linewidth=0.5)
        
        # Optionally draw the robot's orientation and lidar frame
        arrow_length = 1.0
        
        # Draw robot orientation (in red)
        dx_robot = arrow_length * math.cos(robot_yaw)
        dy_robot = arrow_length * math.sin(robot_yaw)
        plt.arrow(odom_x, odom_y, dx_robot, dy_robot, 
                 head_width=0.1, head_length=0.15, fc='red', ec='red', label='Robot Heading')
        
        # Draw lidar orientation (in orange)
        lidar_direction = robot_yaw + lidar_offset
        dx_lidar = arrow_length * math.cos(lidar_direction)
        dy_lidar = arrow_length * math.sin(lidar_direction)
        plt.arrow(odom_x, odom_y, dx_lidar, dy_lidar, 
                 head_width=0.1, head_length=0.15, fc='black', ec='black', label='Lidar Direction')
        
        print("Done drawing rays")
    
    def is_pose_in_map_points(self, pose_stamped, map_points, tolerance=0.01):
        """Check if a PoseStamped exists in map points within tolerance"""
        pose_x = pose_stamped.pose.position.x
        pose_y = pose_stamped.pose.position.y
        
        for x, y in map_points:
            distance = math.sqrt((pose_x - x)**2 + (pose_y - y)**2)
            if distance < tolerance:
                return True, (x, y)
        return False, None

# Then in your setup method:
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

    def visual_path_goals(self):

        """Takes self.global_path and self.local_goals and plots them
        """
        
        if self.global_path == None:
            return
        plt.figure(figsize=(8,6))
        active_obstacles = self.obstacle_manager_.get_active_obstacles()

        for val in self.global_path.poses:
            plt.plot(val.pose.position.x, val.pose.position.y, marker='o', linestyle='-',markersize=3,color='blue')

        for goal in self.local_goal_manager_.data:
            plt.plot(goal.pose.position.x, goal.pose.position.y, marker='o', linestyle='-',markersize=5,color='red')
        if active_obstacles:
            for obstacle in active_obstacles:
                plt.plot(obstacle.center_x, obstacle.center_y, marker='o', linestyle='-',markersize=5,color='green')
        else:
            for obstacle in self.obstacle_manager_.obstacle_array:
                plt.plot(obstacle.center_x, obstacle.center_y, marker='o', linestyle='-',markersize=5,color='green')

        plt.show()

    def extract_messages(self, bag_path, topic):
        """Extract messages from a ROS 2 bag and store them in a dictionary grouped by timestamp."""
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        allowed_topics = {topic}
        # Dictionary to group messages by timestamp
        grouped_data = defaultdict(dict)

        while reader.has_next():
            topic, msg, timestamp = reader.read_next()
            
            if topic not in allowed_topics:
                continue
            # Deserialize message
            msg_type = get_message(type_map[topic])
            msg_deserialized = deserialize_message(msg, msg_type)
            if topic == "/odom":
        # Extract x, y from position
                x = msg_deserialized.pose.pose.position.x
                y = msg_deserialized.pose.pose.position.y

                odom_v = msg_deserialized.twist.twist.linear.x
                odom_w = msg_deserialized.twist.twist.angular.z
                # Extract orientation quaternion and convert to yaw
                qx = msg_deserialized.pose.pose.orientation.x
                qy = msg_deserialized.pose.pose.orientation.y
                qz = msg_deserialized.pose.pose.orientation.z
                qw = msg_deserialized.pose.pose.orientation.w
                yaw = R.from_quat([qx, qy, qz, qw]).as_euler('xyz')[2]

                grouped_data.setdefault(timestamp, {}).update({ ## add getting local velocity and local angular velocity
                    "odom_x": x,
                    "odom_y": y,
                    "odom_yaw": yaw,
                    "odom_v": odom_v,
                    "odom_w": odom_w
                })
            elif topic == "/scan":
                # Convert LaserScan ranges to individual columns
                range_data = list(msg_deserialized.ranges)

                # Store each range as a separate column with an indexed key
                for i, value in enumerate(range_data):
                    grouped_data.setdefault(timestamp, {}).update({
                        f"scan_range_{i}": value
                    })

                
            elif topic == "/cmd_vel":
                v = msg_deserialized.linear.x
                w = msg_deserialized.angular.z
                print(f" v value {v}, w {w}")

                grouped_data.setdefault(timestamp, {}).update({
                    "cmd_v": v, 
                    "cmd_w": w
                    })
        return grouped_data

    def save_to_csv(self, bag_path, output_csv, topic):
        """Converts extracted messages to CSV format."""
        
        messages = self.extract_messages(bag_path, topic) 

        if not messages:
            print("No messages found in the bag file.")
            return

        # Convert dictionary to Pandas DataFrame
        df = pd.DataFrame.from_dict(messages, orient="index")

        # Reset index to turn timestamp into a column
        df.reset_index(inplace=True)
        df.rename(columns={'index': 'timestamp'}, inplace=True)

        df.to_csv(output_csv, index=False)
        print(f"Saved {len(df)} messages to {output_csv}")
    def ray_tracing(self, pose):
        """
        Args: Takes the yaw, and the obstacle data 
        Output: Lidar data with 1080 values
        """
        local_data = [0] * self.NUM_LIDAR
        #print(f" len of active obstacles {len(active_obstacles)}")
        yaw = self.get_yaw(pose)
        incrementor = (2 * math.pi) / self.NUM_LIDAR
        active_obstacles = self.obstacle_manager_.get_active_obstacles_with_sampling(self.global_path, yaw )
        lidar_offset = -math.pi/2 
        for index in range(self.NUM_LIDAR):
            # Calculate direction vector for this ray
            theta_prev = incrementor * index
            theta = yaw + lidar_offset +  theta_prev
            theta = self.normalize_angle(theta)
            dx = math.cos(theta)
            dy = math.sin(theta)
            
            # Initialize to max distance to find closest
            min_distance = float('inf')
            
            # Test against each obstacle
            for obs in active_obstacles:
                # Get obstacle data
                Cx = obs.center_x
                Cy = obs.center_y
                radius = obs.radius
                
                # Compute ray-circle intersection
                mx = pose.position.x - Cx
                my = pose.position.y - Cy
                
                a = dx * dx + dy * dy
                b = 2.0 * (mx * dx + my * dy)
                c = mx * mx + my * my - radius * radius
                
                # Compute discriminant
                discriminant = b * b - 4.0 * a * c
                
                if discriminant >= 0.0:
                    # Has intersection(s)
                    sqrt_discriminant = math.sqrt(discriminant)
                    t1 = (-b - sqrt_discriminant) / (2.0 * a)
                    t2 = (-b + sqrt_discriminant) / (2.0 * a)
                    
                    # Find closest valid intersection
                    if t1 > 0.0 and t1 < min_distance:
                        min_distance = t1
                        
                    if t2 > 0.0 and t2 < min_distance:
                        min_distance = t2
            
            # If we found an intersection, update the distances array
            if min_distance != float('inf'):
                self.distances[index] = min_distance
                local_data[index] = min_distance
        self.get_logger().info("Calculated distances") 
        return local_data 
    def ray_data_append(self, filename=None):

        if filename is None:
            filename = self.lidar_file
        output_dir = os.path.dirname(filename)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        
        with open(filename, 'a', newline='') as csvfile:  # Changed 'w' to 'a' for append mode
            writer = csv.writer(csvfile)
            
            # Write data row - just the ray distances
            writer.writerow(self.distances)  # Simplified to write the ray_data directly
    def get_yaw(self, pose: Pose) -> float:
        quat = (pose.orientation.x, pose.orientation.y,pose.orientation.z,
                pose.orientation.w)
        _, _, yaw = euler_from_quaternion(quat)
        return yaw


    def normalize_angle(self, value):
            return (value + math.pi) % (2 * math.pi) - math.pi
    

    def oversample_cmdVel3(self, odom_csv, cmd_csv, output_csv):
        import pandas as pd

        # Read the CSV files for odom and cmd
        odom_df = pd.read_csv(odom_csv)
        cmd_df = pd.read_csv(cmd_csv)

        # Convert timestamps to numeric for accurate merging
        odom_df['timestamp'] = pd.to_numeric(odom_df['timestamp'])
        cmd_df['timestamp']  = pd.to_numeric(cmd_df['timestamp'])
        
        print("have grabbed values")
        # Merge the command velocities onto odom timestamps using merge_asof.
        # We only keep the timestamp from odom, and the cmd_v and cmd_w from the cmd DataFrame.
        merged_df = pd.merge_asof(
            odom_df[['timestamp']],  # Use only the odom timestamp
            cmd_df[['timestamp', 'cmd_v', 'cmd_w']],
            on='timestamp',
            direction='nearest'
        )

        print("saving to csv")
        # Save only the timestamp, cmd_v, and cmd_w columns to the output CSV
        merged_df.to_csv(output_csv, index=False)
        return merged_df


def main(args=None):
    rclpy.init(args=args)
    test_node = MapTraining()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
