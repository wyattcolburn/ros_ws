
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
import numpy as np
import os
import math
# Declare arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),

    DeclareLaunchArgument('world', default_value='barn_world_0',
                          description='Ignition World'), DeclareLaunchArgument('model', default_value='standard',
                                                                               choices=[
                                                                                   'standard', 'lite'],
                                                                               description='Turtlebot4 Model'),

    DeclareLaunchArgument('map_file', default_value='yaml_0.yaml',
                          description='Map file for localization'),
    DeclareLaunchArgument('auto_start_nav', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically start navigation sequence'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():


    # Paths
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])


    # Include localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('map', LaunchConfiguration('map_file')),
            ('use_sim_time', 'true')
        ]
    )

    # Include nav2 with delay
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true')
        ]
    )

    publish_features_node = Node(
        package='publish_features',
        executable='publish_features_node',
        name='publish_features',  # Fixed: was 'publish'
        output='log',
        parameters=[],
    )

    middle_man_node = Node(
        package='middle_man',
        executable='middle_man_barn',
        name='middle_man',  # Fixed: was 'publish'
        output='screen',
        parameters=[
            {'obstacle_config': '/home/mobrob/ros_ws/config.yaml'}
        ],
    )

    # Add delay for nav2 to start after localization

    delayed_local = TimerAction(
        period=10.0,
        actions=[localization])

    delayed_nav2 = TimerAction(
        period=10.0,  # 3 second delay like your sleep command
        actions=[nav2]
    )
    delayed_publish_node = TimerAction(
        period=15.0,
        actions=[publish_features_node]
    )

    delayed_middle_man_node = TimerAction(
        period=25.0,
        actions=[middle_man_node]
    )
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(undock_with_delay)
    ld.add_action(delayed_local)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_publish_node)

    return ld
