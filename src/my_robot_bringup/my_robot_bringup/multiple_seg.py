"""
How does this run,
We will run test node, which will take the raw motor cmds and publish them so that the robot moves with randomness,
We need to already know how the robot runs to generate the plan, to create the local goals, etc. But to validate we
want to be able to run, so similar to what we are already doing we create CSV files with odom data, could do the same with obstacle data?


Lets say we are odom_rad (x,y), we need to know yaw, and locations of revalent obstacles,  

April 29: Currently, this takes odom_csv, and creates the local goals and obstacles to generate ray traces

"""
import pickle
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

class Segment():

    def __init__(self, map_points, node, RADIUS, OFFSET, start_index=None,end_index=None):

        self.map_points = map_points
        self.node = node
        self.RADIUS = RADIUS
        self.OFFSET = OFFSET
        self.start_index =  start_index
        self.end_index = end_index
    def init(self):
        print("ininting")
        print(f"len of map points {len(self.map_points)}")
        self.local_goal_manager_ = Local_Goal_Manager(self.map_points)
        self.obstacle_manager_= Obstacle_Manager(self.OFFSET, self.RADIUS, self.local_goal_manager_)
        self.global_path = self.create_path()
        self.local_goal_manager_.global_path = self.global_path

        print(f"len of global path {len(self.global_path.poses)}")
        self.create_local_goals()
        self.create_obstacles()

    def create_path(self):
        return self.node.create_path_from_points(self.map_points) 
    def create_local_goals(self):
        self.local_goal_manager_.generate_local_goals_claude(self.global_path) 
        print(f"created the local_goals : {len(self.local_goal_manager_.data)}")
    def create_obstacles(self):
        self.obstacle_manager_.create_all_obstacle()
        print(f"created the obstacles {len(self.obstacle_manager_.obstacle_array)}")

    def get_obstacles(self, current_pos):
        return self.obstacle_manager_.get_active_obstacles_claude(self.global_path, current_pos)


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
    def get_active_obstacles(self):
        current_local_goal_count = self.local_goal_manager_.get_local_goal_counter()
        total_goals = len(self.local_goal_manager_.data)
        print(f"local goal count {current_local_goal_count}") 
        valid_border_min = max(0, current_local_goal_count - 4)
        valid_border_max = min(total_goals, current_local_goal_count + 20)
        active_list = self.obstacle_array[valid_border_min: valid_border_max]
        return active_list
    def get_active_obstacles_claude(self, global_path=None, current_index=None):
        """Select obstacles based on distance to the current robot position."""
        # Get current robot position
        if current_index is not None:
            robot_pos = (self.local_goal_manager_.global_path.poses[current_index].pose.position.x, self.local_goal_manager_.global_path.poses[current_index].pose.position.y)
        else:
            robot_pos = self.local_goal_manager_.current_odom
        #robot_pos = self.local_goal_manager_.current_odom
        print(f"active obstacles reference to where it is {robot_pos}") 
        # Calculate distance to each obstacle
        obstacles_with_distance = []
        for obs in self.obstacle_array:
            dist = math.sqrt((obs.center_x - robot_pos[0])**2 + 
                             (obs.center_y - robot_pos[1])**2)
            obstacles_with_distance.append((obs, dist))
        
        # Sort by distance and take closest N obstacles
        obstacles_with_distance.sort(key=lambda x: x[1])
        active_obstacles = [obs for obs, dist in obstacles_with_distance[:20] if dist < 5.0]
       
        if global_path is not None:
            end_index = min(current_index + 200, len(global_path.poses))
            path_points = [] 
            for i in range(current_index, end_index):
                pose = global_path.poses[i]
                path_points.append((pose.pose.position.x, pose.pose.position.y))
            
            valid_obs = []
            THRESHOLD = .6
            for obs in active_obstacles:
                min_dist =  self.calculate_min_distance_to_path(obs.center_x, obs.center_y, path_points)
                if min_dist > THRESHOLD:
                    valid_obs.append(obs)
                    continue
                else:
                    continue

                
            
            return valid_obs 
        else:
            return active_obstacles
    def calculate_min_distance_to_path(self, x, y, path_points):
        """Calculate minimum distance from point (x,y) to the path."""
        min_dist = float('inf')
        
        for i in range(len(path_points) - 1):
            # Get points for this path segment
            x1, y1 = path_points[i]
            x2, y2 = path_points[i + 1]
            
            # Calculate distance to this segment
            dist = self.point_to_segment_distance(x, y, x1, y1, x2, y2)
            
            # Update minimum distance
            min_dist = min(min_dist, dist)
        
        return min_dist
    def point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        """Calculate distance from point to line segment."""
        # Line segment vector
        dx = x2 - x1
        dy = y2 - y1
        
        # If segment is just a point
        if dx == 0 and dy == 0:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        # Calculate projection parameter
        t = ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)
        
        # Constrain t to segment bounds
        t = max(0, min(1, t))
        
        # Calculate closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        # Return distance to closest point
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

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
        base_threshold = 0.2
        print(f"len of global_path : {len(self.global_path.poses)}") 
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
            #print(f"accumulated_distance is {accumulated_distance}") 
            accumulated_distance += segment_distance
            if accumulated_distance >= adaptive_threshold:
                self.data.append(current_pose)
                accumulated_distance = 0   

        print(f"created local goals: {len(self.data)}")
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
        self.RADIUS = .4
        self.NUM_VALID_OBS = 20
        self.NUM_LIDAR = 1080
        self.Obstacle_list = []

        self.Obstacle_list = []
        self.dist_between_goals = .2
        # Delay the odom-to-map conversion until TF is ready
        self.odom_timer = self.create_timer(2.0, self.check_tf_and_run)


        
        self.distances = [0] * self.NUM_LIDAR

        self.lidar_header_flag = True
        # Files for training data to be stored
        self.input_bag = "/home/wyattcolburn/ros_ws/rosbag2_2025_05_19-18_26_30/"
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

    
        self.segment_setup()
        #self.setup()
        self.get_logger().info(f"Transformed {len(self.map_x)} points to map frame.")


    def visualize_path_with_yaw(self, sample_rate=20):
        """
        Create a visualization of the path with yaw arrows every sample_rate positions.
        
        Args:
            sample_rate: Plot yaw arrows every sample_rate positions
        """
        output_folder = "path_with_yaw"
        os.makedirs(output_folder, exist_ok=True)
        
        plt.figure(figsize=(12, 8))
        ax = plt.gca()
        ax.set_aspect('equal')
        
        # Plot the full path
        path_x = [point[0] for point in self.map_points]
        path_y = [point[1] for point in self.map_points]
        plt.plot(path_x, path_y, '-', color='blue', linewidth=1.0, alpha=0.7, label='Path')
        
        # Extract and plot yaw arrows
        yaw_length = 0.3  # Length of the arrow
        
        for i in range(0, len(self.global_path.poses), sample_rate):
            pos = self.global_path.poses[i].pose.position
            orientation = self.global_path.poses[i].pose.orientation
            _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, 
                                               orientation.z, orientation.w])
            
            # Plot position point
            plt.plot(pos.x, pos.y, 'o', color='black', markersize=3)
            
            # Plot yaw arrow
            dx = yaw_length * math.cos(yaw)
            dy = yaw_length * math.sin(yaw)
            plt.arrow(pos.x, pos.y, dx, dy, 
                     head_width=0.08, head_length=0.15, 
                     fc='red', ec='red')
            
            # Optionally add text labels for angles
            if i % (sample_rate * 5) == 0:  # Add labels less frequently
                angle_degrees = math.degrees(yaw) % 360
                plt.text(pos.x + dx + 0.05, pos.y + dy + 0.05, 
                        f"{angle_degrees:.1f}°", 
                        fontsize=8, color='darkred')
        
        plt.title(f'Path with Yaw Angles (every {sample_rate} positions)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        
        # Save the visualization
        plt.savefig(f"{output_folder}/path_with_yaw.png", dpi=300)
        plt.close()
        
        print(f"Saved path with yaw visualization to {output_folder}/path_with_yaw.png")
    def segment_setup(self):

        self.map_points = list(zip(self.map_x, self.map_y))

        # Global stuff for training data

        self.global_path = self.create_path_from_points(self.map_points)

        self.local_goal_manager_ = Local_Goal_Manager((self.map_points[0][0], self.map_points[0][1]))

        self.local_goal_manager_.global_path = self.global_path
        self.local_goal_manager_.generate_local_goals_claude(self.global_path)

        self.local_goal_manager_.upscale_local_goal((self.local_goal_manager_.data[0].pose.position.x, self.local_goal_manager_.data[0].pose.position.y), self.map_points, self.local_goals_output)

        self.current_odom = (self.map_points[0][0], self.map_points[0][1])
        self.segments = self.create_segments(self.map_points)

        for seg in self.segments:
            print(f"segment index {seg.start_index} and {seg.end_index}")
            print(f"len of seg.map_points : {len(seg.map_points)}")
        self.debug_segment_obstacles()
        print("done")
        self.process_segment(self.segments[0], 0)


    def debug_segment_obstacles(self, seg_index=0):  # Use a small segment
        seg = self.segments[seg_index]
        
        plt.figure(figsize=(12, 8))
        
        # Plot segment path
        path_x = [p[0] for p in seg.map_points]
        path_y = [p[1] for p in seg.map_points]
        plt.plot(path_x, path_y, 'b-', linewidth=2, label='Segment Path')
        
        # Plot local goals for this segment
        lg_x = [lg.pose.position.x for lg in seg.local_goal_manager_.data]
        lg_y = [lg.pose.position.y for lg in seg.local_goal_manager_.data]
        plt.scatter(lg_x, lg_y, c='green', s=50, label='Local Goals')
        
        # Plot ALL obstacles for this segment
        for obs in seg.obstacle_manager_.obstacle_array:
            circle = plt.Circle((obs.center_x, obs.center_y), obs.radius, 
                               fill=False, color='red', linewidth=1)
            plt.gca().add_patch(circle)
        
        # Test obstacle filtering at middle of segment
        mid_index = len(seg.map_points) // 2
        active_obs = seg.get_obstacles(mid_index)
        
        # Highlight active obstacles
        for obs in active_obs:
            circle = plt.Circle((obs.center_x, obs.center_y), obs.radius, 
                               fill=False, color='orange', linewidth=3)
            plt.gca().add_patch(circle)
        
        plt.title(f'Segment {seg_index} - Red=All Obstacles, Orange=Active')
        plt.legend()
        plt.axis('equal')
        plt.grid(True)
        plt.show()
        
        print(f"Total obstacles: {len(seg.obstacle_manager_.obstacle_array)}")
        print(f"Active obstacles: {len(active_obs)}")
        #self.process_segment(self.segments[0], 0)
        
        #for i, seg in enumerate(self.segments):
        #    print(f"checking start and end index : {seg.start_index} and {seg.end_index}")
        #    self.process_segment(seg, i)
        #self.main_loop() 

    def process_all_segments(self, segments):
        # but process in one loop and then return then process, 
        for seg_index, seg in enumerate(segments):
            self.process_segment(seg, seg_index)
     

    def process_segment(self, seg, seg_index):

        """
        Args: Segment
        Output: Nothing, data just written to CSV
        Make a dkr
        First, make odom_data.csv, cmd_vel_output.csv, local_goals.csv just for that section, 
        ray_trace
        """
        
        self.lidar_header_flag = True
        output_folder = f"{self.input_bag}/seg_{seg_index}_test_ind/input_data"
        os.makedirs(output_folder, exist_ok=True)

        odom_all = pd.read_csv(self.odom_csv_file)
        odom_curr = odom_all[seg.start_index:seg.end_index]
        odom_curr.to_csv(f"{output_folder}/odom_data.csv")
        
        cmd_all = pd.read_csv(self.cmd_output_csv)
        cmd_curr = cmd_all[seg.start_index:seg.end_index]
        cmd_curr.to_csv(f"{output_folder}/cmd_vel_output.csv")

        local_goal_all = pd.read_csv(self.local_goals_output)
        local_goal_curr = local_goal_all[seg.start_index:seg.end_index]
        local_goal_curr.to_csv(f"{output_folder}/local_goals.csv")
       
        if os.path.exists(f"{output_folder}/lidar_data.csv"):
            os.remove(f"{output_folder}/lidar_data.csv")

        print(f"odom_all length: {len(odom_all)}")
        print(f"cmd_all length: {len(cmd_all)}")
        print(f"local_goal_all length: {len(local_goal_all)}")
        
        print(f"Reading odom from: {self.odom_csv_file}")
        print(f"Reading cmd from: {self.cmd_output_csv}")  
        print(f"Reading local_goals from: {self.local_goals_output}")

        # Add these debug prints in process_segment:
        odom_all = pd.read_csv(self.odom_csv_file)
        odom_curr = odom_all[seg.start_index:seg.end_index]
        print(f"Original odom shape: {odom_all.shape}")
        print(f"Sliced odom shape: {odom_curr.shape}")
        print(f"First few rows of sliced odom:")
        print(odom_curr.head())

        # Same for local_goals
        local_goal_all = pd.read_csv(self.local_goals_output)
        local_goal_curr = local_goal_all[seg.start_index:seg.end_index]
        print(f"Original local_goals shape: {local_goal_all.shape}")
        print(f"Sliced local_goals shape: {local_goal_curr.shape}")
        print(f"First few rows of sliced local_goals:")
        print(local_goal_curr.head())
        self.current_odom_index = 0
        counter = 0

        path_x = [point[0] for point in seg.map_points]
        path_y = [point[1] for point in seg.map_points]

        frames_folder = f"{output_folder}/frames"
        os.makedirs(frames_folder, exist_ok=True)
    
        for i, map_point in enumerate(seg.map_points[:len(cmd_curr)]):
            
            self.current_odom = (map_point[0], map_point[1])
            seg.local_goal_manager_.current_odom = self.current_odom
            active_obstacles = seg.get_obstacles(i)
            ray_data = self.ray_tracing(seg.global_path.poses[i].pose,seg,active_obstacles)
            self.ray_data_append(filename=f"{output_folder}/lidar_data.csv")

            if i % 500 != 0:
                counter +=1
                self.current_odom_index +=1
                continue

            plt.clf()  # clear previous plot
            ax = plt.gca()
            ax.set_aspect('equal')
            # replot the base elements
            plt.plot(self.current_odom[0], self.current_odom[1], marker='o', linestyle='-', markersize=3, color='blue', label="odometry path")


            print("local goal count") 
            for obstacle in active_obstacles:
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

            # plot the entire path
            plt.plot(path_x, path_y, marker='o', linestyle='-', markersize=3, color='black', label='odom path')
            self.draw_rays_claude_2(self.current_odom[0], self.current_odom[1], ray_data, seg)
            # save the frame
            
            frame_path = f"{frames_folder}/frame_{counter:03d}.png"
            counter+=1
            self.current_odom_index +=1
            plt.savefig(frame_path)
    
        plt.close()
        print("done with main loop")
                
                

    def main_loop(self):
   
        """
        I want to take in a list of segments, I want to create data sets for each of them
        """
        output_folder = "may14_test"
        os.makedirs(output_folder, exist_ok=True)
        plt.figure(figsize=(8, 6))
        counter = 0

        path_x = [point[0] for point in self.map_points]
        path_y = [point[1] for point in self.map_points]
        for j, segment in enumerate(self.segments):
            self.current_odom_index = 0
            for i, map_point in enumerate(segment.map_points):
                self.current_odom = (map_point[0], map_point[1])

                segment.local_goal_manager_.current_odom = self.current_odom
                active_obstacle = segment.get_obstacles(i)
                local_data = self.ray_tracing(segment.global_path.poses[i].pose,segment, active_obstacle) # get ray tracining values, now need to store them correctly with 

                self.ray_data_append()
                if i % 50 != 0:
                    counter +=1
                    self.current_odom_index +=1
                    continue

                plt.clf()  # clear previous plot
                ax = plt.gca()
                ax.set_aspect('equal')
                # replot the base elements
                plt.plot(self.current_odom[0], self.current_odom[1], marker='o', linestyle='-', markersize=3, color='blue', label="odometry path")


                print("local goal count") 
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

                # plot the entire path
                plt.plot(path_x, path_y, marker='o', linestyle='-', markersize=3, color='black', label='odom path')
                self.draw_rays_claude_2(self.current_odom[0], self.current_odom[1], local_data, segment)
                # save the frame
                
                frame_path = f"{output_folder}/frame_{counter:03d}.png"
                counter+=1
                self.current_odom_index +=1
                plt.savefig(frame_path)
        
            plt.close()
            print("done with main loop")


    def save_segments_to_csv(self, segments, filename="segments_cache.csv"):
        """Save segment information to a CSV file"""
        print(f"Saving {len(segments)} segments to {filename}...")
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Write header row
            writer.writerow(["segment_id", "start_index", "end_index"])
            # Write data for each segment
            for i, segment in enumerate(segments):
                writer.writerow([f"seg_{i}", segment.start_index, segment.end_index])
        print("Segments saved successfully")
    def create_segments(self, map_points):
        """
        Create segments which act like little global paths, so need to create obstacles for just that segment

        """
        print("starting create_segments")
        segments_data = [] 
        if os.path.exists(f"{self.input_bag}/segments_cache.csv"):
            print("values already calcualted")
            
            with open(f"{self.input_bag}/segments_cache.csv", 'r') as file:
                csv_reader = csv.reader(file)
                
                # Skip the header row
                header = next(csv_reader)
                
                # Read each row and append to list
                for row in csv_reader:
                    start_index = int(row[1])
                    end_index = int(row[2])
                    curr_seg_ = Segment(self.map_points[start_index:end_index],self, self.RADIUS, self.OFFSET)
                    curr_seg_.start_index = start_index
                    curr_seg_.end_index = end_index
                    curr_seg_.init()
                    
                    # Add the row data as a tuple or list to segments_data
                    segments_data.append(curr_seg_)

            for seg in segments_data:
                print(f"seg {seg.start_index} end {seg.end_index}")

        else:
            # Or if you prefer a dictionary:
            # segments_data.append({'segment_id': segment_id, 'start_index': start_index, 'end_index': end_index})
            print("values not cached already")
            current_segment = [map_points[0]]
            start_index = 0
            end_index = 0
            threshold = .15
            for i in range(1, len(map_points)):

                current_segment.append(map_points[i])
                start_index 

                for j in range(len(current_segment) - 200):
                    if self.distance_between_points(current_segment[j], map_points[i]) < threshold:
                        end_index = i - 1
                        curr_seg_ = Segment(current_segment, self, self.RADIUS, self.OFFSET, start_index, end_index)
                        curr_seg_.init()
                        start_index = i
                        segments_data.append(curr_seg_)
                        current_segment = [map_points[i]]
                        
                        break
            if current_segment:

                print(f"last segment : start index {start_index} and end {len(map_points)-1}")
                print("*************************************")
                curr_seg_ = Segment(current_segment, self, self.RADIUS, self.OFFSET, start_index, (len(map_points)-1))
                curr_seg_.init()
                segments_data.append(curr_seg_)
                
            self.save_segments_to_csv(segments_data, f"{self.input_bag}/segments_cache.csv")
        self.plot_segments(segments_data)

        return segments_data
    def plot_segments(self, segments):
        plt.figure(figsize=(10, 8))
        
        # Define a colormap to get distinct colors for each segment
        colors = plt.cm.jet(np.linspace(0, 1, len(segments)))
        
        for i, segment in enumerate(segments):
            path_x = [point[0] for point in segment.map_points]
            path_y = [point[1] for point in segment.map_points]
            
            # Plot each segment with a different color and add to legend
            plt.plot(path_x, path_y, marker='o', linestyle='-', markersize=3, 
                     color=colors[i], label=f'Segment {i+1}')
            
            # Optionally mark the start and end of each segment
            plt.plot(path_x[0], path_y[0], 'go', markersize=6)  # Start point (green)
            plt.plot(path_x[-1], path_y[-1], 'ro', markersize=6)  # End point (red)
        
        plt.title('Path Segments')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.legend(loc='best')
        plt.tight_layout()
        plt.show()
        return plt.gcf()  # Return the figure if you want to save it later
    def draw_rays_claude_2(self, odom_x, odom_y, lidar_readings, segment):
        # Get robot's yaw from the current pose (you need to pass this as a parameter)
        robot_yaw = self.get_yaw(segment.global_path.poses[self.current_odom_index].pose)
        
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

    def ray_tracing(self, pose, segment, active_obstacles):
        """
        Args: Takes the yaw, and the obstacle data 
        Output: Lidar data with 1080 values
        """
        local_data = [0] * self.NUM_LIDAR
        #active_obstacles = self.test_seg.obstacle_manager_.get_active_obstacles_claude(self.global_path, self.current_odom_index)

        print(f"I am located at {(pose.position.x, pose.position.y)}")
        #print(f" len of active obstacles {len(active_obstacles)}")
        yaw = self.get_yaw(pose)
        incrementor = (2 * math.pi) / self.NUM_LIDAR
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
            if self.lidar_header_flag:
                headers = [f'lidar_{i}' for i in range(len(self.distances))]
                writer.writerow(headers)
                self.lidar_header_flag = False
            
            # Write data row - just the ray distances
            writer.writerow(self.distances)   

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
