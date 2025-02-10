import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import math

class obstacle:
    def __init__(self, cx, cy, x, y):
        self.x = x
        self.y = y
        self.cx = cx
        self.cy = cy

obstacles_list = []
def draw_circle(cx, cy, r, odom_x, odom_y, color='green', linestyle='-', linewidth=2):

    """
    Draws a circle given the center (cx, cy) and radius r.

    Parameters:
    - cx, cy: Center coordinates of the circle.
    - r: Radius of the circle.
    - color: Color of the circle outline.
    - linestyle: Line style for the circle (default is '-').
    - linewidth: Line width of the circle.
    """

    for x, y in zip(odom_x, odom_y):
        distance_sq = (x - cx) ** 2 + (y - cy) ** 2
        if distance_sq <= r ** 2:  # If point is inside or on the boundary
            return False  # Overlapping points found


    theta = np.linspace(0, 2 * np.pi, 640)  # 100 points for smooth circle
    x = cx + r * np.cos(theta)
    y = cy + r * np.sin(theta)
    current_obstacle = obstacle(cx, cy, x,y)
    obstacles_list.append(current_obstacle)


    plt.plot(x, y, color=color, linestyle=linestyle, linewidth=linewidth)

    # Set axis equal for proper aspect ratio
    plt.axis("equal")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Circle Plot")

# If no points are inside the circle
    return True

def ray_trace(obstacle, current_odom_x, current_odom_y):
    
    radians_per_index = (2*np.pi) /640
    num_lidar = 640
    hallucinated_lidar = [0] * 640
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
            hallucinated_lidar[i] = distance
        else:
            print("max value")
            hallucinated_lidar[i] = 0.0
    
    
    return hallucinated_lidar
def visualize_odom(csv_file, output_file=None):
    # Load data from CSV
    df = pd.read_csv(csv_file)

    # Extract only x and y positions
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()

    df_lg = pd.read_csv("local_goals_max.csv")

    local_goals_x = df_lg['odom_x'].tolist()
    local_goals_y = df_lg['odom_y'].tolist()
    local_goals_yaw = df_lg['odom_yaw'].tolist()

    # Convert yaw angles to unit vectors for quiver
    arrow_length = 0.1  # Adjust the arrow length as needed
    dx = arrow_length * np.cos(local_goals_yaw)
    dy = arrow_length * np.sin(local_goals_yaw)

    #dx_pi = arrow_length * np.cos(local_goals_yaw + np.pi)
    #dy_pi = arrow_lenght * np.sin(local_goals_yaw + np_pi)

    # Create figure
    plt.figure(figsize=(8, 6))

    # Plot odometry path (without yaw)
    plt.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")

    # Plot local goals
    plt.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")

    # Add arrows representing yaw at each local goal
    plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")
    #plt.quiver(local_goals_x, local_goals_y, dx_pi, dy_pi, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw opposite")


    # Add legend for the path
    plt.legend(loc="best")

    # Labels and grid
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Odometry Path Visualization")
    plt.grid(True)

    threshold = .5
    print(len(local_goals_x))
    for i in range(0,1):
            #len(local_goals_x)):
        print(f"current yaw {local_goals_yaw[i]} opposite yaw {local_goals_yaw[i] + math.pi}") 
        if draw_circle(local_goals_x[i]+ threshold * math.cos(local_goals_yaw[i]),local_goals_y[i]+threshold * math.sin(local_goals_yaw[i]),.15, odom_x, odom_y): print("first circle success")
        #elif draw_circle(local_goals_x[i]+ threshold * math.cos(local_goals_yaw[i]+ math.pi),local_goals_y[i]+threshold * math.sin(local_goals_yaw[i]+math.pi),.1, odom_x, odom_y):
            #print("draw on opposite side") 
        else:
            print("circle overlapps")
    print(obstacles_list[0].x, obstacles_list[0].y)

    hallucinated_reading = ray_trace(obstacles_list[0], odom_x[0], odom_y[0])
    print(hallucinated_reading)
    # Save or show plot
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

