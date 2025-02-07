import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import math


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


    theta = np.linspace(0, 2 * np.pi, 100)  # 100 points for smooth circle
    x = cx + r * np.cos(theta)
    y = cy + r * np.sin(theta)

    plt.plot(x, y, color=color, linestyle=linestyle, linewidth=linewidth)

    # Set axis equal for proper aspect ratio
    plt.axis("equal")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.title("Circle Plot")

# If no points are inside the circle
    return True
def visualize_odom(csv_file, output_file=None):
    # Load data from CSV
    df = pd.read_csv(csv_file)

    # Extract only x and y positions
    odom_x = df['odom_x'].tolist()
    odom_y = df['odom_y'].tolist()

    df_lg = pd.read_csv("local_goal.csv")

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
    plt.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='green', label="Local Goals Yaw")

    # Add legend for the path
    plt.legend(loc="best")

    # Labels and grid
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Odometry Path Visualization")
    plt.grid(True)

    threshold = .25
    print(len(local_goals_x))
    for i in range(len(local_goals_x)):
        if draw_circle(local_goals_x[i]+ threshold * math.cos(local_goals_yaw[i]),local_goals_y[i]+threshold * math.sin(local_goals_yaw[i]),.1, odom_x, odom_y):
            continue
        else:
            print("circle overlapps")
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

