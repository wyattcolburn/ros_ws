import time
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import math
import csv
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

    for i in range(len(odom_x)):

        for lidar_counter in range(640):

            ang = lidar_counter * (2*np.pi / 640)
            distance = lidar_readings[i][lidar_counter]
            current_x = odom_x[i]
            current_y = odom_y[i]

            projection_x = current_x + distance * math.cos(ang)
            projection_y = current_y + distance * math.sin(ang)
            plt.plot([current_x, projection_x], [current_y,projection_y], linestyle='solid', color='green', label="Ray Trace")

    
def draw_ray(odom_x, odom_y, lidar_readings):


    for lidar_counter in range(640):

        ang = lidar_counter * (2*np.pi / 640)
        distance = lidar_readings[lidar_counter]
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
    # Load data from CSV
    df = pd.read_csv(csv_file)

    # Extract only x and y positions
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()

    df_lg = pd.read_csv("local_hall.csv")

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

    obstacles = np.zeros((70, 640,2))
    obstacle_counter = 0
    # Labels and grid
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Odometry Path Visualization")
    plt.grid(True)

    threshold = .2
    print(len(local_goals_x))
         
    for i in range(len(local_goals_x)-1): #dont need an obstacle for each odom, just local goals
        obstacles, obstacle_counter = perp_circle((local_goals_x[i], local_goals_y[i]), (local_goals_x[i + 1], local_goals_y[i + 1]), .1, .4, obstacles, obstacle_counter)
    
    lidar_readings = load_lidar_rays("output_perp_2.csv")
    print(f"len of {len(lidar_readings)} {len(odom_x)}")
    for i in range(10):
        draw_ray(odom_x[i], odom_y[i], lidar_readings[i])
        time.sleep(1)
    if output_file:
        plt.savefig(output_file)
        print(f"Plot saved to {output_file}")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="Visualize odometry data from a CSV file.")
    parser.add_argument("csv_file", type=str, help="Input CSV file containing odometry data.")
    parser.add_argument("--output", type=str, default=None, help="Output file to save the plot (optional).")

    args = parser.parse_args()
    visualize_odom(args.csv_file, args.output)

if __name__ == "__main__":
    main()

