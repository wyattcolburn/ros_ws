from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
import numpy as np
import os
# Declare arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='barn_world_0',
                          description='Ignition World'), DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    
    DeclareLaunchArgument('map_file', default_value='yaml_0.yaml',
                          description='Map file for localization'),
    DeclareLaunchArgument('world_num', default_value='0',
                          description='What trial?'),
    DeclareLaunchArgument('auto_start_nav', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically start navigation sequence'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

    

def path_coord_to_gazebo_coord(x, y):
    # Tis is from the jackal_timer github repo from dperille (UT-AUSTIN LAB)
    RADIUS=.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)

def generate_launch_description():
    
    world_num = int(os.environ.get('WORLD_NUM', '0'))
    world_name = f'world_{world_num}' 
    starting_location_path = np.load(os.path.expanduser(f'~/ros_ws/BARN_turtlebot/path_files/path_{world_num}.npy'))[0]
    start_location_gazebo = path_coord_to_gazebo_coord(starting_location_path[0], starting_location_path[1])
    
    map_file_path = os.path.expanduser(f'~/ros_ws/BARN_turtlebot/map_files/yaml_{world_num}.yaml')
    
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_my_robot_bringup = get_package_share_directory(
        'my_robot_bringup')
    # Paths
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
    
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
            ('x', f'{start_location_gazebo[0]}'),
            ('y', f'{start_location_gazebo[1]}'), 
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )

    undock_command = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '/undock', 'irobot_create_msgs/action/Undock', '{}'],
        output='screen'
    )
    undock_with_delay = TimerAction(
        period=25.0,  # Delay in seconds
        actions=[undock_command]
    )

    """
    The idea is that the bash script will say which trial we are running. This will be decided by world_name number,
    then we need to provide the .yaml for the map file, we also then to grab the path values, and convert them

    also need this for start position

    So POC:
        1) Spawn into the world, at the path[0] converted spawn, 
        2) Load the localization,
        3) NAV with DWA
    """

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
    
    barn_one_shot = Node(
        package='my_robot_bringup',
        executable='barn_one_shot',
        name='one_shot_trial',
        output='screen',
        parameters=[{
            'initial_x':  float(start_location_gazebo[0]),
            'initial_y': float(start_location_gazebo[1]),
            'initial_yaw': float(3.14),
            'wait_after_undock': 2.0,
            'pose_init_delay': 1.0,
            'world_num': world_num,
        }]
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
        parameters=[],
    )
    
    # Add delay for nav2 to start after localization
    delayed_nav2 = TimerAction(
        period=3.0,  # 3 second delay like your sleep command
        actions=[nav2]
    )
    delayed_barn= TimerAction(
        period=25.0,  # 3 second delay like your sleep command
        actions=[barn_one_shot]
    )
    delayed_publish_node = TimerAction(
        period=15.0,
        actions=[publish_features_node]
    )
    
    delayed_middle_man_node = TimerAction(
        period=15.0,
        actions=[middle_man_node]
    )
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    # ld.add_action(undock_with_delay)
    ld.add_action(localization)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_publish_node)
    ld.add_action(delayed_middle_man_node)
    ld.add_action(delayed_barn)
    return ld

