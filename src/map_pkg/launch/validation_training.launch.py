
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define package paths
    turtlebot4_ignition_bringup_pkg = FindPackageShare('turtlebot4_ignition_bringup')
    turtlebot4_navigation_pkg = FindPackageShare('turtlebot4_navigation')
    
    # Include the simulator launch
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot4_ignition_bringup_pkg,
                'launch',
                'turtlebot4_ignition.launch.py'
            ])
        ])
    )
    
    # Include the localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot4_navigation_pkg,
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={
            'map': '/home/wyattcolburn/ros_ws/big_map_april_4.yaml',
            'use_sim_time': 'true'
        }.items()
    )
    
    # Include the navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                turtlebot4_navigation_pkg,
                'launch',
                'nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    # Create and return launch description
    return LaunchDescription([
        simulator_launch,
        localization_launch,
        navigation_launch
    ])

