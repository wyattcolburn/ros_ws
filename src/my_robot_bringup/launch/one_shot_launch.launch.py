from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.conditions import IfCondition

# Declare arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='maze',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    # Navigation sequence parameters
    DeclareLaunchArgument('initial_x', default_value='-.250',
                          description='Initial X position'),
    DeclareLaunchArgument('initial_y', default_value='0.0',
                          description='Initial Y position'),
    DeclareLaunchArgument('initial_yaw', default_value='3.141',
                          description='Initial yaw orientation'),
    DeclareLaunchArgument('goal_x', default_value='-1.5',
                          description='Goal X position'),
    DeclareLaunchArgument('goal_y', default_value='0.5',
                          description='Goal Y position'),
    DeclareLaunchArgument('goal_yaw', default_value='3.141',
                          description='Goal yaw orientation'),
    DeclareLaunchArgument('auto_start_nav', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically start navigation sequence'),
    DeclareLaunchArgument('map_file', default_value='exp1.yaml',
                          description='Map file for localization'),
    # Optional package flags
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot spawn pose.'))


def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_my_robot_bringup = get_package_share_directory(
        'my_robot_bringup')
    
    # Get additional package directories
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    
    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
    
    # Include ignition simulation
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # Include robot spawn
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
    
    # Add delay for nav2 to start after localization
    delayed_nav2 = TimerAction(
        period=3.0,  # 3 second delay like your sleep command
        actions=[nav2]
    )

    one_shot = Node(
        package='my_robot_bringup',
        executable='one_shot',
        name='one_shot_trial',
        output='screen',
        parameters=[{
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
            'wait_after_undock': 2.0,
            'pose_init_delay': 1.0,
        }]
    )

    # Optional nodes - only launch if packages exist and are enabled
    publish_features_node = Node(
        package='publish_features',
        executable='publish_features_node',
        name='publish_features',  # Fixed: was 'publish'
        output='log',
        parameters=[],
    )
    
    middle_man_node = Node(
        package='middle_man',
        executable='middle_man_valid',
        name='middle_man',  # Fixed: was 'publish'
        output='log',
        parameters=[],
    )
    
    # Add delays
    delayed_one_shot_node = TimerAction(
        period=15.0,
        actions=[one_shot]
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
    ld.add_action(localization)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_one_shot_node)
    ld.add_action(delayed_publish_node)
    ld.add_action(delayed_middle_man_node)
    
    return ld
