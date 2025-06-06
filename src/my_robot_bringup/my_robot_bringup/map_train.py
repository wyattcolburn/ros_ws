"""
How does this run,
We will run test node, which will take the raw motor cmds and publish them so that the robot moves with randomness,
We need to already know how the robot runs to generate the plan, to create the local goals, etc. But to validate we
want to be able to run, so similar to what we are already doing we create CSV files with odom data, could do the same with obstacle data?


Lets say we are odom_rad (x,y), we need to know yaw, and locations of revalent obstacles,  
"""


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

from nav_msgs.msg import Odometry 
from visualization_msgs.msg import MarkerArray

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
    def get_active_obstacles(self):
        current_local_goal_count = self.local_goal_manager_.get_local_goal_counter()
        print("local goal count")      
        active_list = self.obstacle_array[current_local_goal_count:min(len(self.obstacle_array), current_local_goal_count + 19)]
        return active_list


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

        dir_x = current_local_goal.pose.position.x - next_local_goal.pose.position.x
        dir_y = current_local_goal.pose.position.y - next_local_goal.pose.position.y
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
        ob1.center_x = current_local_goal.pose.position.x + offset_x
        ob1.center_y = current_local_goal.pose.position.y + offset_y
        ob1.radius = self.RADIUS

        ob2 = Obstacle()
        ob2.center_x = current_local_goal.pose.position.x - offset_x
        ob2.center_y = current_local_goal.pose.position.y - offset_y
        ob2.radius = self.RADIUS
       
        self.obstacle_array.append(ob1)
        self.obstacle_array.append(ob2)


class Local_Goal_Manager():
    def __init__(self, current_odom, global_path = None):
        self.data = []
        self.global_path = global_path
        self.current_lg_counter = 0
        self.current_odom = current_odom
        self.current_lg = None
    def generate_local_goals(self):
        """
        Generate local goals along the global path based on the current robot position
        """
        if self.global_path is None:
            print("Cannot generate local goals: No global path available")
            return

        # Find the closest point on the path to the current robot position

        # Start from the closest point and generate local goals along the path

        
        accumulated_distance = 0.0
        threshold_distance = .05

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
    def update(self):
        # Comparing poses so might not work
        if self.global_path is None or self.current_lg is None or self.current_odom[0] is None:
            return
        dist_to_goal = self.distance_between_points(self.current_lg, self.current_odom)
        if self.current_odom == self.current_lg or dist_to_goal < .13:
            self.current_lg_counter += 1
            self.current_lg = (self.data[self.current_lg_counter].pose.position.x, self.data[self.current_lg_counter].pose.position.y)
            print("Have updated current local goal")
        else:
            print(f"have yet to reach local goal: dist to lg {dist_to_goal}")
        
    def get_local_goal_counter(self):
        return self.current_lg_counter
    def get_coords(self):

        return (self.current_lg[0], self.current_lg[1])

    def set_global_path(self, new_global_path):
        if new_global_path is not None and len(new_global_path.poses) > 0:
            self.global_path = new_global_path
            self.current_lg_counter = 0
            self.current_lg = (self.global_path.poses[3].pose.position.x, self.global_path.poses[3].pose.position.y)
            return True
        return False

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
        
        self.current_odom = (0.0, 0.0)
        
        self.OFFSET = 2.0
        self.RADIUS = .5
        self.NUM_VALID_OBS = 20
        self.NUM_LIDAR = 1080
        self.Obstacle_list = []

        self.Obstacle_list = []
        self.dist_between_goals = .2
        self.current_odom = (None, None)
        # Delay the odom-to-map conversion until TF is ready
        self.odom_timer = self.create_timer(2.0, self.check_tf_and_run)
        self.local_goal_manager_ = Local_Goal_Manager(self.current_odom)
        self.obstacle_manager_ = Obstacle_Manager(self.OFFSET, self.RADIUS,self.local_goal_manager_)
        # Still keep the live timer for example/demo
        # Sub for lidar data


        
        self.distances = [0] * self.NUM_LIDAR
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10) 
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.hall_pub_ = self.create_publisher(LaserScan, '/HallScan', 10)


        self.motor_cmd_file = "/home/wyattcolburn/model/test1/input_data/cmd_vel_output.csv"
        self.motor_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.motor_timer = self.create_timer(0.05, self.publish_motor_cmd)  # 10Hz
        # Publisher for lidar data
   
        self.obstacle_marker_pub = self.create_publisher(MarkerArray, '/hallucinated_obstacles', 10)
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.local_goals_pub = self.create_publisher(MarkerArray, '/local_goals', 10)

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

        self.setup()
        self.get_logger().info(f"Transformed {len(self.map_x)} points to map frame.")

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
        if self.distances and len(self.distances) == 1080:
            try:
                spoofed_scan.ranges = [float(x) for x in self.distances]
            except Exception as e:
                self.get_logger().error(f"Failed to convert distances to float: {e}")
                spoofed_scan.ranges = [0.0] * 1080
                print("publishing hall lidar")
        else:
            spoofed_scan.ranges = [0.0] * 1080
            print("Not working yet")
        self.hall_pub_.publish(spoofed_scan)

    def setup(self):
        self.map_points = list(zip(self.map_x, self.map_y))
        self.global_path = self.create_path_from_points(self.map_points)
        self.local_goal_manager_.set_global_path(self.global_path)
        self.local_goal_manager_.generate_local_goals()
        self.get_logger().info(f"len of local_goal data within node {len(self.local_goal_manager_.data)}")
        self.obstacle_manager_.create_all_obstacle()
    
    def main_loop(self):
        self.local_goal_manager_.update()
        self.ray_tracing(self.global_path.poses[0].pose)
        self.publish_visualization() 
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

    def ray_tracing(self, pose):
        """
        Args: Takes the yaw, and the obstacle data 
        Output: Lidar data with 1080 values
        """
        active_obstacles = self.obstacle_manager_.get_active_obstacles()
        print(f" len of active obstacles {len(active_obstacles)}")
        yaw = self.get_yaw(pose)
        incrementor = (2 * math.pi) / self.NUM_LIDAR
        start_angle= yaw - math.pi/2 
        for index in range(self.NUM_LIDAR):
            # Calculate direction vector for this ray
            theta_prev = start_angle + incrementor * index
            theta = self.normalize_angle(theta_prev)
            
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
        self.get_logger().info("Calculated distances") 
    def get_yaw(self, pose: Pose) -> float:
        quat = (pose.orientation.x, pose.orientation.y,pose.orientation.z,
                pose.orientation.w)
        _, _, yaw = euler_from_quaternion(quat)
        return yaw


    def normalize_angle(self, value):
            return (value + math.pi) % (2 * math.pi) - math.pi
# Add this method
    def publish_motor_cmd(self):
        if not hasattr(self, 'motor_cmds'):
            # Load motor commands on first run
            self.motor_cmds = pd.read_csv(self.motor_cmd_file)
            self.motor_cmd_index = 0
            self.get_logger().info(f"Loaded {len(self.motor_cmds)} motor commands")
        
        if self.motor_cmd_index < len(self.motor_cmds):
            cmd = Twist()
            cmd.linear.x = float(self.motor_cmds.iloc[self.motor_cmd_index]['cmd_v'])
            cmd.angular.z = float(self.motor_cmds.iloc[self.motor_cmd_index]['cmd_w'])
            self.motor_pub.publish(cmd)
            self.publish_visualization()
            self.motor_cmd_index += 1
        else:
            self.get_logger().info("Finished publishing all motor commands")
            self.motor_timer.cancel()
        if self.global_path:
            self.main_loop()
    def publish_visualization(self):
        # Publish path
        if self.global_path:
            self.path_pub.publish(self.global_path)
        
        # Publish obstacles as markers
        marker_array = MarkerArray()
        active_obstacles = self.obstacle_manager_.get_active_obstacles()
        
        for i, obs in enumerate(active_obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obs.center_x
            marker.pose.position.y = obs.center_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = obs.radius * 2
            marker.scale.y = obs.radius * 2
            marker.scale.z = 0.5
            marker.color.a = 0.7
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        
        self.obstacle_marker_pub.publish(marker_array)
        
        # Publish local goals
        goals_marker_array = MarkerArray()
        for i, goal in enumerate(self.local_goal_manager_.data):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "local_goals"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = goal.pose.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            goals_marker_array.markers.append(marker)
        
        self.local_goals_pub.publish(goals_marker_array)
    def odom_callback(self, msg):

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        
        coords = odom_to_map(self, odom_x, odom_y)
        if coords is None:
            return                      # skip this cycle gracefully
        map_x, map_y = coords
        self.current_odom = (map_x, map_y)

        self.local_goal_manager_.current_odom = self.current_odom  

def main(args=None):
    rclpy.init(args=args)
    test_node = MapTraining()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
