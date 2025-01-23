import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader

# Path to your ROS bag file
bag_path = Path('/home/wyattcolburn/ros_ws/bags/2025_01_23-11_39_54')

# Topic to visualize
lidar_topic = '/scan'

# Open the bag file and read messages
with AnyReader([bag_path]) as reader:
    # Filter connections for the lidar topic
    connections = [x for x in reader.connections if x.topic == lidar_topic]

    # Process lidar messages
    for connection, _, raw_message in reader.messages(connections):
        # Use AnyReader's built-in deserialize method
        message = reader.deserialize(raw_message, connection.msgtype)

        # Extract lidar ranges and angles
        ranges = np.array(message.ranges)  # Lidar ranges
        angle_min = message.angle_min  # Starting angle
        angle_increment = message.angle_increment  # Angle step

        # Compute angles
        angles = angle_min + np.arange(len(ranges)) * angle_increment

        # Convert to Cartesian coordinates for visualization
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Plot the lidar scan
        plt.figure()
        plt.scatter(x, y, s=1)
        plt.title('Lidar Scan')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.axis('equal')
        plt.show()

