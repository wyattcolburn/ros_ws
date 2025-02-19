import time
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import math
import csv
import os
import glob
import cv2
import multiprocessing as mp


def load_lidar_rays(csv_file):
    """Loads LiDAR data from a CSV file and returns it as a list of lists."""
    lidar_values = []
    try:
        with open(csv_file, "r") as file:
            reader = csv.reader(file)
            for row in reader:
                lidar_values.append([float(value) for value in row])  # Convert to float
        print(f"Loaded {len(lidar_values)} LiDAR scans from {csv_file}")
    except Exception as e:
        print(f"Error loading CSV: {e}")
    return lidar_values


def draw_rays(odom_x, odom_y, lidar_readings):
    print("i am in the function of draw rays")
    for i in range(len(odom_x)):

        for lidar_counter in range(640):

            ang = lidar_counter * (2*np.pi / 640)
            distance = lidar_readings[i][lidar_counter]
            current_x = odom_x[i]
            current_y = odom_y[i]

            projection_x = current_x + distance * math.cos(ang)
            projection_y = current_y + distance * math.sin(ang)
            plt.plot([current_x, projection_x], [current_y,projection_y], linestyle='solid', color='green')

    
def draw_ray(odom_x, odom_y, lidar_readings):

    print("i am in func")
    for lidar_counter in range(640):

        ang = lidar_counter * (2*np.pi / 640)
        distance = lidar_readings[lidar_counter]
        if distance == 0:
            continue
        current_x = odom_x
        current_y = odom_y
        projection_x = current_x + distance * math.cos(ang)
        projection_y = current_y + distance * math.sin(ang)
        plt.plot([current_x, projection_x], [current_y,projection_y], linestyle='solid', color='green', label="Ray Trace")

def perp_circle(p1, p2, radius, offset_x, obstacles, obstacle_counter):
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
    circle_x = cx + radius * np.cos(theta)
    circle_y = cy + radius * np.sin(theta)
    plt.plot(circle_x, circle_y, 'r-', label="Perpendicular Circle")
    

    obstacles[obstacle_counter] = np.column_stack((circle_x, circle_y))
    obstacle_counter += 1
    theta = np.linspace(0, 2 * np.pi, 640)
    circle_x2 = cx2 + radius * np.cos(theta)
    circle_y2 = cy2 + radius * np.sin(theta)

    obstacles[obstacle_counter] = np.column_stack((circle_x2, circle_y2))
    obstacle_counter += 1
    plt.plot(circle_x2, circle_y2, 'r-', label="Perpendicular Circle")

    return obstacles, obstacle_counter 


def visualize_odom(csv_file, output_file=None):


    df = pd.read_csv(csv_file)

    # Extract only x and y positions
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()

    df_lg = pd.read_csv("test_new_lg_0.csv")

    local_goals_x = df_lg['odom_x'].tolist()
    local_goals_y = df_lg['odom_y'].tolist()
    local_goals_yaw = df_lg['odom_yaw'].tolist()

# Convert yaw angles to unit vectors for quiver
    arrow_length = 0.1  # Adjust the arrow length as needed
    dx = arrow_length * np.cos(local_goals_yaw)
    dy = arrow_length * np.sin(local_goals_yaw)


    # Create figure
    plt.figure(figsize=(8, 6))

    small_odom_x, small_odom_y = odom_x[0:99], odom_y[0:99]
    # Plot odometry path (without yaw)
    plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")

    #plt.plot(small_odom_x, small_odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
    # Plot local goals
    plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")

    # Add arrows representing yaw at each local goal
    plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")


    # Add legend for the path
    plt.legend(loc="best")
    dy = arrow_length * np.sin(local_goals_yaw)


    # Create figure
    plt.figure(figsize=(8, 6))

    small_odom_x, small_odom_y = odom_x[0:99], odom_y[0:99]
    # Plot odometry path (without yaw)
    plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")

    #plt.plot(small_odom_x, small_odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
    # Plot local goals
    plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")

    # Add arrows representing yaw at each local goal
    plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")


    # Add legend for the path
    plt.legend(loc="best")

    obstacles = np.zeros((len(local_goals_x)*2, 640,2))
    obstacle_counter = 0
    # Labels and grid
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Odometry Path Visualization")
    plt.grid(True)

    lidar_readings = load_lidar_rays("test_lidar_0.csv")

    generate_frames_parallel(odom_x, odom_y, local_goals_x, local_goals_y, dx, dy, lidar_readings, .2, .7, output_folder="ray_frames_parallel")

def generate_frames(odom_x, odom_y, local_goals_x, local_goals_y, dx, dy, lidar_readings, obstacles, obstacle_counter, output_folder="ray_frames"):
    """
    Generates and saves individual frames for LIDAR visualization.
    """
    os.makedirs(output_folder, exist_ok=True)  # Ensure the folder exists

    plt.figure(figsize=(8, 6))
    print(f"len of number obstacles {len(obstacles)}")
    for i in range(len(odom_x)):
        plt.clf()  # Clear previous plot
        
        # Replot the base elements
        plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
        plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")
        plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")
        
        lg_counter = int(i / len(local_goals_x))
        close_obstacles = np.copy(obstacles[lg_counter:lg_counter + 10])
        for obstacle in close_obstacles:
            plt.plot(obstacle[:, 0], obstacle[:, 1], color='red')

        # Labels, grid, and legend
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Odometry Path Visualization")
        plt.grid(True)
        plt.legend(loc="best")
        # Draw only the current ray
        print(f"odom x {odom_x[i]} odom y {odom_y[i]} i {i}")
        draw_ray(odom_x[i], odom_y[i], lidar_readings)
        
        # Save the frame
        frame_path = f"{output_folder}/frame_{i:03d}.png"
        plt.savefig(frame_path)
        #print(f"Saved {frame_path}")

        #plt.show()        
    plt.close()
def generate_frames_obst(odom_x, odom_y, local_goals_x, local_goals_y, obstacles, obstacle_counter, lidar_readings, output_folder="ray_frames"):
    """
    Generates and saves individual frames for LIDAR visualization.
    """
    os.makedirs(output_folder, exist_ok=True)  # Ensure the folder exists
    newObstacles = []
    prevObstacleTracker = -1
    for odomCounter in range(1000):
        plt.figure(figsize=(8, 6))
        print(f"len of number obstacles {len(obstacles)}")
        plt.clf()  # Clear previous plot
        print(f"length of lidar measurements {len(lidar_readings)}")
        # Replot the base elements
        plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
        plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")
        
        obstacleTracker = int(odomCounter / (len(odom_x) / len(local_goals_x)))

        #print(f"before checkobst tracker : {obstacleTracker} and prev value {prevObstacleTracker} and odomval {odom_counter}")
                # Update newObstacles only if obstacleTracker changes
        if obstacleTracker != prevObstacleTracker:
            #print("obstacle value not equal prev tracker valu **********************************")
            newObstacles = []  # Reset obstacles
            #print(f"tracker val {obstacleTracker*2} and len obstacles - 1 {len(obstacles)-1}") 
            if obstacleTracker * 2 < len(obstacles) - 1:  # Ensure index is in bounds
                #print("new obstacles*******************88")
                newObstacles.append(obstacles[obstacleTracker * 2])
                newObstacles.append(obstacles[obstacleTracker * 2 + 1])
                #print(f"new obstacles********************************** {newObstacles}")
            prevObstacleTracker = obstacleTracker  # Update tracker

        for obstacle in newObstacles:
            dist_to_obstacle =  math.sqrt((obstacle.centerPoint[0] - odom_x[odomCounter]) ** 2 + (obstacle.centerPoint[1]  - odom_y[odomCounter]) ** 2)   
            if dist_to_obstacle < 1: 
                plt.plot(obstacle.x_points, obstacle.y_points, color='red')

            else:
                continue
                
        print("after plotting obstacles")
        # Labels, grid, and legend
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Odometry Path Visualization")
        plt.grid(True)
        plt.legend(loc="best")
        draw_ray(odom_x[odomCounter], odom_y[odomCounter], lidar_readings[odomCounter]) 
        

        # Save the frame
        frame_path = f"{output_folder}/frame_{odomCounter:03d}.png"
        plt.savefig(frame_path)
        print(f"Saved frame: {frame_path}")  # Debugging print statement
        plt.close()

def generate_frames_parallel(odom_x, odom_y, local_goals_x, local_goals_y, dx, dy, lidar_readings, obstacle_radius, obstacle_offset, output_folder="ray_frames"):
    """
    Parallelized version of generate_frames using multiprocessing.
    """
    os.makedirs(output_folder, exist_ok=True)

    num_frames = len(odom_x)
    num_workers = min(mp.cpu_count(), num_frames)  # Use max available CPUs

    # Create argument tuples for each frame
    args = [
        (i, odom_x, odom_y, local_goals_x, local_goals_y, dx, dy, lidar_readings, obstacle_radius, obstacle_offset, output_folder)
        for i in range(num_frames)
    ]

    # Use multiprocessing Pool
    with mp.Pool(processes=num_workers) as pool:
        pool.starmap(generate_frames, args)  # Map function to arguments in parallel


def main():
    parser = argparse.ArgumentParser(description="Visualize odometry data from a CSV file.")
    parser.add_argument("csv_file", type=str, help="Input CSV file containing odometry data.")
    parser.add_argument("--output", type=str, default=None, help="Output file to save the plot (optional).")

    args = parser.parse_args()
    visualize_odom(args.csv_file, args.output)
    
if __name__ == "__main__":
    main()

