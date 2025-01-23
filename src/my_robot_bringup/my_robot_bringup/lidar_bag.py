import os
import subprocess

def record_lidar_bag(output_directory, topics):
    """
    Record a ROS2 bag file for lidar data.

    :param output_directory: Path to save the bag file.
    :param topics: List of topics to record.
    """
    try:
        # Create output directory if it doesn't exist
        os.makedirs(output_directory, exist_ok=True)

        # Command to start recording
        cmd = ["ros2", "bag", "record", "-o", output_directory] + topics

        # Start the recording process
        print(f"Recording lidar data to {output_directory} for topics: {topics}")
        subprocess.run(cmd, check=True)

    except Exception as e:
        print(f"Error while recording bag: {e}")


if __name__ == "__main__":
    # Path where the bag file will be saved
    output_directory = "/home/wyattcolburn/ros_ws/ros_bags/lidar_bag"

    # Topics to record
    topics = ["/scan"]  # Add other topics as needed

    # Start recording
    record_lidar_bag(output_directory, topics)

