from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction

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
    DeclareLaunchArgument('initial_x', default_value='0.0',
                          description='Initial X position'),
    DeclareLaunchArgument('initial_y', default_value='0.0',
                          description='Initial Y position'),
    DeclareLaunchArgument('initial_yaw', default_value='3.141',
                          description='Initial yaw orientation'),
    DeclareLaunchArgument('goal_x', default_value='-4.0',
                          description='Goal X position'),
    DeclareLaunchArgument('goal_y', default_value='0.0',
                          description='Goal Y position'),
    DeclareLaunchArgument('goal_yaw', default_value='0.0',
                          description='Goal yaw orientation'),
    DeclareLaunchArgument('auto_start_nav', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically start navigation sequence'),
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
    
    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    
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

    # Reliable navigation sequence node
    reliable_nav_node = Node(
        package='my_robot_bringup',  # Replace with your actual package name
        executable='validate',
        name='reliable_navigation_sequence',
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

    # Add a delay before starting the navigation sequence to ensure everything is ready
    delayed_nav_node = TimerAction(
        period=15.0,  # Increased delay to 15 seconds
        actions=[reliable_nav_node]
    )

    # Optional: ROS2 bag recording (uncomment if needed)
    # bag_record = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '/scan', '/scan_spoofed', '/tf', '/tf_static', '/odom', '/cmd_vel'],
    #     output='screen'
    # )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    ld.add_action(delayed_nav_node)
    # ld.add_action(bag_record)  # Uncomment if you want bag recording
    
    return ld

