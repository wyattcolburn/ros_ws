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



input_bag = "/home/wyattcolburn/ros_ws/utils/rosbag2_2025_02_19-12_49_02"
frame_dkr = "training_complete_test"
odom_csv_file = os.path.join(frame_dkr, "odom_data.csv")
obstacle_radius = .1
obstacle_offset = .4


class Obstacle:
    def __init__(self, cx, cy, x_points, y_points):
        self.centerPoint = (cx,cy)
        self.x_points = x_points
        self.y_points = y_points


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
    # Draw the original line
    plt.plot([odom_x1, odom_x2], [odom_y1, odom_y2], 'b-', label="Original Line")

    # Draw the perpendicular circle
    theta = np.linspace(0, 2 * np.pi, 640)
    circle_x = list(cx + radius * np.cos(theta))
    circle_y = list(cy + radius * np.sin(theta))
  
    


    theta = np.linspace(0, 2 * np.pi, 640)
    circle_x2 = list(cx2 + radius * np.cos(theta))
    circle_y2 = list(cy2 + radius * np.sin(theta))
    

    # Check for intersection and create obstacles
    obstacleOne = Obstacle(cx, cy, circle_x, circle_y) if not intersects_path(cx, cy, radius, odom_x, odom_y) else None
    obstacleTwo = Obstacle(cx2, cy2, circle_x2, circle_y2) if not intersects_path(cx2, cy2, radius, odom_x, odom_y) else None 

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

            grouped_data.setdefault(timestamp, {}).update({
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


def ray_trace(obstacles, odom_x, odom_y, local_goals_x):
    # I want to change this to only factor in a couple of obstacles, 
    radians_per_index = (2 * np.pi) / 640  # LiDAR resolution
    num_lidar = 640
    hallucinated_lidar = np.zeros((len(odom_x), num_lidar), dtype=float)

    for odom_counter in range(len(odom_x)):# Iterate over all odometry points
        current_odom_x = odom_x[odom_counter]
        current_odom_y = odom_y[odom_counter]

        angle_dict = {}  # Store the minimum distance per unique angle
        
        for obstacle_counter in range(len(obstacles)): #what value should this be?
            if obstacles[obstacle_counter]:
                dist_to_obstacle =  math.sqrt((obstacles[obstacle_counter].centerPoint[0] - current_odom_x) ** 2 + (obstacles[obstacle_counter].centerPoint[1]  - current_odom_y) ** 2)   
                if dist_to_obstacle < 3:
                    for i in range(num_lidar): # a loop for each lidar angle
                        current_obstacle_x = obstacles[obstacle_counter].x_points[i]
                        current_obstacle_y = obstacles[obstacle_counter].y_points[i]

                        # Calculate angle of the obstacle relative to the robot
                        angle = math.atan2(current_obstacle_y - current_odom_y, current_obstacle_x - current_odom_x)
                        # Calculate distance
                        distance = math.sqrt((current_odom_x - current_obstacle_x) ** 2 + (current_odom_y - current_obstacle_y) ** 2)
                        if distance > 2: # this shouldnt be an issue because osbtacles overlap
                            distance = 0 #maybe max ranges
                        
                        # Normalize the angle to the closest LiDAR index { rad -> index }
                        if angle < 0:
                            angle = angle + 2 * math.pi
                        angle_index = round(angle / radians_per_index)

                        # Store only the closest obstacle at each angle
                        if angle_index not in angle_dict or distance < angle_dict[angle_index]:
                            angle_dict[angle_index] = distance

                else:
                    continue
            else:
                continue
        # Convert dictionary back into the hallucinated_lidar array
        for angle_index, distance in angle_dict.items():
            if 0 <= angle_index < num_lidar:  # Ensure index is within range
                hallucinated_lidar[odom_counter][angle_index] = distance
        #print(angle_dict)
    return hallucinated_lidar

def draw_ray(odom_x, odom_y, lidar_readings):


    for lidar_counter in range(640):

        ang = lidar_counter * (2*np.pi / 640)
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

                plt.plot(obstacle.x_points, obstacle.y_points, color='red')
                
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
    print(lidar_readings)
    generate_frames_obst(odom_x, odom_y, local_goals_x, local_goals_y, obstacleArray, lidar_readings, frame_dkr) 
if __name__ == "__main__":
    main()
