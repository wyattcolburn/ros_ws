import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_output_dir(context, *args, **kwargs):
    # Generate a unique directory name with a timestamp and milliseconds
    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    base_dir = LaunchConfiguration("base_output_dir").perform(context)
    output_dir = os.path.join(base_dir, timestamp)
    return output_dir


def generate_launch_description():
    # Base output directory argument
    base_output_dir_arg = DeclareLaunchArgument(
        "base_output_dir",
        default_value="/home/wyattcolburn/ros_ws/bags",
        description="Base directory to save lidar bags"
    )

    # Topics to record
    topics_arg = DeclareLaunchArgument(
        "topics",
        default_value="/scan",
        description="Topics to record in the bag"
    )

    # Use OpaqueFunction to dynamically compute the output directory
    def create_record_bag_process(context, *args, **kwargs):
        # Generate output directory with a unique timestamp
        output_dir = generate_output_dir(context)

        # Pass the dynamically generated directory to `ros2 bag record`
        return [
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "-o", output_dir,
                    LaunchConfiguration("topics")
                ],
                output="screen",
                shell=True,
            )
        ]

    # Create the LaunchDescription
    return LaunchDescription([
        base_output_dir_arg,
        topics_arg,
        OpaqueFunction(function=create_record_bag_process),
    ])

