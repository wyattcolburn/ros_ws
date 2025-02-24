import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import rosbag2_py
from collections import defaultdict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
import math
import csv
from matplotlib.patches import Circle

input_bag = "/home/wyattcolburn/ros_ws/utils/rosbag2_2025_02_19-12_12_38"
frame_dkr = "idk_test"
os.makedirs(frame_dkr, exist_ok=True)
odom_csv_file = os.path.join(frame_dkr, "odom_data.csv")
debug_lidar = os.path.join(frame_dkr, "debug.csv")
lidar_file = os.path.join(frame_dkr, "lidar_data.csv")
obstacle_radius = .1
obstacle_offset = .4
num_lidar_points = 1080

class Obstacle:
    def __init__(self, cx, cy):
        self.centerPoint = (cx,cy)

def hall_csv(hallucinated_lidar, output_file):
    with open(output_file, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(hallucinated_lidar)  # Writes all rows at once 

def hall_csv_2(hallucinated_lidar, output_file):
    """
    Write each LiDAR value in a separate row, along with its row index (time step)
    and angle index. This produces a CSV with columns:
        row_index, angle_index, lidar_value
    so that you don't have to scroll horizontally through 1080 columns.
    """
    with open(output_file, "w", newline="") as file:
        writer = csv.writer(file)
        # Optionally write a header:
        writer.writerow(["RowIndex", "AngleIndex", "LidarValue"])

        # hallucinated_lidar is assumed to be shape (num_rows, 1080)
        for row_idx, row_data in enumerate(hallucinated_lidar):
            for angle_idx, lidar_value in enumerate(row_data):
                writer.writerow([row_idx, angle_idx, lidar_value])

#def lidar_pipeline(hallucinated_lidar, odom_csv, output_csv):
#
#    with open(output_file, "w", newline="") as file:
#        writer = csv.writer(file)
#        writer.writeheader("odom
def intersects_path(cx, cy, radius, odom_x, odom_y):
    for ox, oy in zip(odom_x, odom_y):
        dist = math.sqrt((cx - ox) ** 2 + (cy - oy) ** 2)
        if dist <= radius:
            return True  # Intersection detected
    return False
def lg_distance(input_file, output_file, interval, next_target):
    df = pd.read_csv(input_file)

    # Assuming columns ['x', 'y', 'z'] exist
    df['dx'] = df['odom_x'].diff()
    df['dy'] = df['odom_y'].diff()

    # Euclidean distance between points
    df['step_distance'] = np.sqrt(df['dx']**2 + df['dy']**2)
    # Compute cumulative distance
    df['cumulative_distance'] = df['step_distance'].cumsum().fillna(0)



    # Store selected local goals
    local_goals_x = []
    local_goals_y = []

    # Store points at regular distance intervals


    for i, row in df.iterrows():
        if row['cumulative_distance'] >= next_target:
            local_goals_x.append(row['odom_x'])
            local_goals_y.append(row['odom_y'])
            next_target += interval  # Update next target distance
    print(local_goals_x)

    # Create DataFrame
    goals_df = pd.DataFrame({'local_goals_x': local_goals_x, 'local_goals_y': local_goals_y})

    # Save to CSV or display
    goals_df.to_csv(output_file, index=False)

    return local_goals_x, local_goals_y


def perp_circle_array(p1, p2, radius, offset_x, odom_x, odom_y):
    odom_x1, odom_y1 = p1
    odom_x2, odom_y2 = p2
    mx, my = (odom_x1 + odom_x2) / 2 , (odom_y1 + odom_y2) / 2
    if (odom_x2 - odom_x1) == 0:
        print("vertical slope")
        perp_slope = 0
        dx, dy = 0, offset_x
    elif odom_y2 - odom_y1 == 0:
        perp_slope = np.inf
        dx, dy = 0, offset_x
    else:
        slope = (odom_y2 - odom_y1) / (odom_x2 - odom_x1)
        perp_slope = -1 / slope
        mag = np.sqrt(1 + perp_slope **2)
        dx = offset_x / mag
        dy = (offset_x * perp_slope) / mag

    cx, cy = mx + dx, my + dy
    cx2, cy2 = mx - dx, my - dy
    
    # Check for intersection and create obstacles
    obstacleOne = Obstacle(cx, cy) if not intersects_path(cx, cy, radius, odom_x, odom_y) else None
    obstacleTwo = Obstacle(cx2, cy2) if not intersects_path(cx2, cy2, radius, odom_x, odom_y) else None 

    return obstacleOne, obstacleTwo 

def extract_messages(bag_path):
    """Extract messages from a ROS 2 bag and store them in a dictionary grouped by timestamp."""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    allowed_topics = {'/odom'}
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

            # Extract orientation quaternion and convert to yaw
            qx = msg_deserialized.pose.pose.orientation.x
            qy = msg_deserialized.pose.pose.orientation.y
            qz = msg_deserialized.pose.pose.orientation.z
            qw = msg_deserialized.pose.pose.orientation.w
            yaw = R.from_quat([qx, qy, qz, qw]).as_euler('xyz')[2]

            grouped_data.setdefault(timestamp, {}).update({ ## add getting local velocity and local angular velocity
                "odom_x": x,
                "odom_y": y,
                "odom_yaw": yaw
            })
        elif topic == "/scan":
            # Convert LaserScan ranges to individual columns
            range_data = list(msg_deserialized.ranges)

            # Store each range as a separate column with an indexed key
            for i, value in enumerate(range_data):
                grouped_data.setdefault(timestamp, {}).update({
                    f"scan_range_{i}": value
                })

            
    return grouped_data

def save_to_csv(bag_path, output_csv):
    """Converts extracted messages to CSV format."""
    
    messages = extract_messages(bag_path)

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
def ray_circle_intersection(ray_origin_x, ray_origin_y, ray_angle, circle_center_x, circle_center_y, circle_radius):
    """
    Calculate the intersection of a ray with a circle.
    Returns the distance to the closest intersection point, or None if no intersection.
    """
    # Vector from origin to circle center
    dx = circle_center_x - ray_origin_x
    dy = circle_center_y - ray_origin_y
    
    # Ray direction vector
    ray_dir_x = math.cos(ray_angle)
    ray_dir_y = math.sin(ray_angle)
    
    # Project vector to circle center onto ray direction
    a = ray_dir_x * dx + ray_dir_y * dy
    
    # Square of perpendicular distance from circle center to ray
    b = dx * dx + dy * dy - a * a
    
    # If perpendicular distance is greater than radius, no intersection
    if b > circle_radius * circle_radius:
        return None
    
    # Distance from closest point to intersection points
    f = math.sqrt(circle_radius * circle_radius - b)
    
    # Distance to closest intersection point
    t = a - f
    
    # If intersection is behind the ray origin, no valid intersection
    if t < 0:
        return None
        
    return t
def ray_trace(obstacles, odom_x, odom_y, local_goals_x):
    radians_per_index = (2 * np.pi) / num_lidar_points
    hallucinated_lidar = np.zeros((len(odom_x), num_lidar_points), dtype=float)
    
    for odom_counter in range(len(odom_x)):
        current_odom_x = odom_x[odom_counter]
        current_odom_y = odom_y[odom_counter]
        
        # For each LiDAR angle
        for i in range(num_lidar_points):
            angle = i * radians_per_index
            min_distance = float('inf')
            
            # Check intersection with each obstacle
            for obstacle in obstacles:
                if obstacle:  # Check if obstacle exists
                    # Get circle properties (assuming obstacles have centerPoint and radius attributes)
                    center_x = obstacle.centerPoint[0]
                    center_y = obstacle.centerPoint[1]
                    
                    # Calculate distance to obstacle center
                    dist_to_center = math.sqrt((center_x - current_odom_x)**2 + 
                                             (center_y - current_odom_y)**2)
                    
                    # Only check nearby obstacles (optimization)
                    if dist_to_center < 5 + obstacle_radius:
                        intersection_dist = ray_circle_intersection(
                            current_odom_x, current_odom_y, angle,
                            center_x, center_y, obstacle_radius
                        )
                        
                        if intersection_dist is not None and intersection_dist < min_distance:
                            min_distance = intersection_dist
            
            # Store the closest intersection distance
            if min_distance != float('inf'):
                hallucinated_lidar[odom_counter][i] = min_distance
                
    return hallucinated_lidar

def draw_ray(odom_x, odom_y, lidar_readings):


    for lidar_counter in range(len(lidar_readings)):

        ang = lidar_counter * (2*np.pi / num_lidar_points)
        distance = lidar_readings[lidar_counter]
        current_x = odom_x
        current_y = odom_y

        projection_x = current_x + distance * math.cos(ang)
        projection_y = current_y + distance * math.sin(ang)
        plt.plot([current_x, projection_x], [current_y,projection_y], linestyle='solid', color='green', label="Ray Trace")


def generate_frames_obst(odom_x, odom_y, local_goals_x, local_goals_y, obstacles, lidar_readings, output_folder="ray_frames"):
    """
    Generates and saves individual frames for LIDAR visualization.
    """
    os.makedirs(output_folder, exist_ok=True)  # Ensure the folder exists
    newObstacles = []
    prevObstacleTracker = -1
    print(f"len of lidar_readings {lidar_readings}")
    for odomCounter in range(0,len(odom_x),5):
        plt.figure(figsize=(8, 6))
        print(f"len of number obstacles {len(obstacles)}")
        plt.clf()  # Clear previous plot
        print(f"length of lidar measurements {len(lidar_readings)}")
        # Replot the base elements
        plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
        plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")
        
        obstacleTracker = int(odomCounter / (len(odom_x) / len(local_goals_x)))
        
        for obstacle in obstacles:
            if obstacle:
                circle = Circle((obstacle.centerPoint[0], obstacle.centerPoint[1]), obstacle_radius, fill=False, color='r')
                plt.gca().add_patch(circle)
                
        print("after plotting obstacles")
        # Labels, grid, and legend
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Odometry Path Visualization")
        plt.grid(True)
        plt.legend(loc="best")
        draw_ray(odom_x[odomCounter], odom_y[odomCounter], lidar_readings[odomCounter]) 
        
        plt.plot(odom_x[odomCounter], odom_y[odomCounter], marker= '|', markersize=9, color='yellow', label="Current Odom")
        # Save the frame
        frame_path = f"{output_folder}/frame_{odomCounter:03d}.png"
        plt.savefig(frame_path)
        print(f"Saved frame: {frame_path}")  # Debugging print statement
        
    plt.close()

def main():

    save_to_csv(input_bag, odom_csv_file) # turned bag into csv
    local_goals_x, local_goals_y = lg_distance(odom_csv_file, "lg_dist.csv", .1, 0)

    df = pd.read_csv(odom_csv_file)

    # Extract only x and y positions
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()


    # Convert yaw angles to unit vectors for quiver
    arrow_length = 0.1  # Adjust the arrow length as needed


    # Create figure
    plt.figure(figsize=(8, 6))

    # Plot odometry path (without yaw)
    plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")

    # Plot local goals
    plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")

    # Add arrows representing yaw at each local goal
    #plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")


    # Add legend for the path
    plt.legend(loc="best")

    
        
    print(len(local_goals_x))
    
    obstacleArray = []
    obstacleArrayCounter = 0
    for i in range(len(local_goals_x)-1): #dont need an obstacle for each odom, just local goals
        obstacleOne, obstacleTwo = perp_circle_array((local_goals_x[i], local_goals_y[i]), (local_goals_x[i + 1], local_goals_y[i + 1]), obstacle_radius, obstacle_offset, odom_x, odom_y)
        obstacleArray.append(obstacleOne)
        obstacleArray.append(obstacleTwo)
    lidar_readings = ray_trace(obstacleArray, odom_x, odom_y, local_goals_x) 
    hall_csv(lidar_readings, lidar_file)
    hall_csv_2(lidar_readings[0:1], debug_lidar)
    generate_frames_obst(odom_x, odom_y, local_goals_x, local_goals_y, obstacleArray, lidar_readings, frame_dkr) 
if __name__ == "__main__":
    main()
