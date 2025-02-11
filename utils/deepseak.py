import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Load odometry data
df = pd.read_csv("odom_hall.csv")
odom_x = df['odom_x'].tolist()
odom_y = df['odom_y'].tolist()

# Load local goals
df_lg = pd.read_csv("local_hall.csv")
local_goals_x = df_lg['odom_x'].tolist()
local_goals_y = df_lg['odom_y'].tolist()
local_goals_yaw = df_lg['odom_yaw'].tolist()

# Convert yaw angles to unit vectors for quiver
arrow_length = 0.1
dx = arrow_length * np.cos(local_goals_yaw)
dy = arrow_length * np.sin(local_goals_yaw)

# Load LiDAR readings
lidar_readings = np.loadtxt("output_perp_2.csv", delimiter=",").reshape(-1, 2)

# Create figure and axis
fig, ax = plt.subplots(figsize=(8, 6))

# Plot static elements (odometry, local goals, arrows)
ax.plot(odom_x, odom_y, marker='o', linestyle='-', markersize=3, color='blue', label="Odometry Path")
ax.plot(local_goals_x, local_goals_y, marker='o', linestyle='-', markersize=3, color='red', label="Local Goals")
ax.quiver(local_goals_x, local_goals_y, dx, dy, angles='xy', scale_units='xy', scale=1, color='black', label="Local Goals Yaw")

# Labels and grid
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_title("Odometry Path Visualization")
ax.legend(loc="best")
ax.grid(True)

# Initialize LiDAR plot
lidar_plot, = ax.plot([], [], 'g.', markersize=2, label="LiDAR Rays")

def update(frame):
    """Update function for animation: Refresh LiDAR readings"""
    if frame >= len(lidar_readings):
        return lidar_plot,

    # Get current LiDAR readings
    lidar_x, lidar_y = lidar_readings[frame][:, 0], lidar_readings[frame][:, 1]

    # Update LiDAR plot
    lidar_plot.set_data(lidar_x, lidar_y)

    return lidar_plot,

# Create animation (15 FPS)
ani = animation.FuncAnimation(fig, update, frames=len(lidar_readings), interval=1000/15, blit=True)

plt.show()

