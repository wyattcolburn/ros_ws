import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import math
import csv
class obstacle:
    def __init__(self, cx, cy, x, y):
        self.x = x
        self.y = y
        self.cx = cx
        self.cy = cy


def ray_trace(obstacle, odom_x, odom_y):
    
    radians_per_index = (2*np.pi) /640
    num_lidar = 640
    hallucinated_lidar = np.zeros((len(odom_x),640), dtype=float)

    for odom_counter in range(len(odom_x)):
        current_odom_x = odom_x[odom_counter]
        current_odom_y = odom_y[odom_counter]

        for i in range(num_lidar):

            current_obstacle_x = obstacle.x[i]
            current_obstacle_y = obstacle.y[i]

            #slope calculation
            slope = (current_odom_y - current_obstacle_y ) / (current_odom_x - current_obstacle_x)

        #distance calc 
            distance = math.sqrt((current_odom_x - current_obstacle_x) ** 2 + (current_odom_y - current_obstacle_y) **2 )
            print(f"slope : {slope} and distance : {distance}")
            value = slope % radians_per_index

            if math.isclose(value, 0, abs_tol = 1E-3):
                print("hit, append to array")
                plt.scatter(current_obstacle_x, current_obstacle_y, color='black', marker='|', s=100, label='Obstacle')

                # Plot the ray from odometry position to the obstacle
                plt.plot([current_odom_x, current_obstacle_x], [current_odom_y, current_obstacle_y], linestyle='dashed', color='red', label="Ray Trace")
                hallucinated_lidar[odom_counter][i] = distance
            else:
                print("max value")
                hallucinated_lidar[odom_counter][i] = 0.0
    
    
    return hallucinated_lidar

def hall_csv(hallucinated_lidar, output_file):
    with open("hallucinated_lidar.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(hallucinated_lidar)  # Writes all rows at once 
    print("written to file")



def perp_circle(p1, p2, radius, offset_x, obstacles, obstacle_counter):
    odom_x1, odom_y1 = p1
    odom_x2, odom_y2 = p2
    print(f"odom x1 {odom_x1}, odom x2 {odom_x2}")
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

    obstacles[obstacle_counter] = (circle_x, circle_y)
    obstacle_counter += 1
    theta = np.linspace(0, 2 * np.pi, 640)
    circle_x2 = cx2 + radius * np.cos(theta)
    circle_y2 = cy2 + radius * np.sin(theta)

    obstacles[obstacle_counter] = (circle_x2, circle_y2)
    obstacle_counter += 1
    plt.plot(circle_x2, circle_y2, 'r-', label="Perpendicular Circle")



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

    # Plot odometry path (without yaw)
    plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")

    # Plot local goals
    plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")

    # Add arrows representing yaw at each local goal
    plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")


    # Add legend for the path
    plt.legend(loc="best")

    obstacles = np.zeros((70, 2,640))
    obstacle_counter = 0
    # Labels and grid
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Odometry Path Visualization")
    plt.grid(True)

    threshold = .2
    print(len(local_goals_x))
    
    # creating the path
    for i in range(len(local_goals_x)-1):
        print(f"lg counter val: {i}") 
        print(f"odom x1/x2 {odom_x[i]} and {odom_x[i+1]}")
        perp_circle((odom_x[i], odom_y[i]), (odom_x[i + 1], odom_y[i + 1]), .1, .2, obstacles, obstacle_counter)


    if output_file:
        plt.savefig(output_file)
        print(f"Plot saved to {output_file}")
    else:
        plt.show()
    #hall_csv(hallucinated_reading, "lidar_output.csv") 
def main():
    parser = argparse.ArgumentParser(description="Visualize odometry data from a CSV file.")
    parser.add_argument("csv_file", type=str, help="Input CSV file containing odometry data.")
    parser.add_argument("--output", type=str, default=None, help="Output file to save the plot (optional).")

    args = parser.parse_args()
    visualize_odom(args.csv_file, args.output)

if __name__ == "__main__":
    main()

