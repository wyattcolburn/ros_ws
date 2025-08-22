from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
import subprocess
import os
from datetime import datetime
import yaml
# Declare arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='sandbox',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_my_robot_bringup = get_package_share_directory(
        'my_robot_bringup')
    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    random_walk_node = TimerAction(
        period=15.0,
        actions=[Node(
            package='my_robot_bringup',
            executable='gaussian',  # You need to create this
            output='screen')]

    )

    bag_dkr = "default_bags"  # Default fallback
    filepath = os.path.join(os.path.expanduser('~'), 'ros_ws', 'config.yaml')
    with open(filepath, "r") as file:
        config = yaml.safe_load(file)

        bag_dkr = config["BAG_DKR"]  # Example access

    base_dir = os.path.join(os.path.expanduser(
        '~'), 'ros_ws', 'ros_bag', bag_dkr)
    os.makedirs(base_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    bag_output = os.path.join(base_dir, f'{ts}_gaus')

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/scan', '/scan_spoofed', '/tf', '/tf_static',
            '/odom', '/cmd_vel', '/clock',
            '-o', bag_output
        ],
        output='screen'
    )

    # Add delay if needed
    delayed_bag_record = TimerAction(
        period=5.0,  # Start recording after robot is ready
        actions=[bag_record]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    ld.add_action(random_walk_node)
    ld.add_action(delayed_bag_record)
    return ld
