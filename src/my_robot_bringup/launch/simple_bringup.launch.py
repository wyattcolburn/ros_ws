from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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
    DeclareLaunchArgument('world', default_value='sandbox',
                          description='Ignition World'), DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('map_file', default_value='sandbox.yaml',
                          description='Map file for localization'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn_nodock.launch.py'])
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
    
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', '-2.0'),
            ('y', '1.0'),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw'))]
    )
    # robot_spawn = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([robot_spawn_launch]),
    #     launch_arguments=[
    #         ('namespace', LaunchConfiguration('namespace')),
    #         ('rviz', LaunchConfiguration('rviz')),
    #         ('x', LaunchConfiguration('x')),
    #         ('y', LaunchConfiguration('y')),
    #         ('z', LaunchConfiguration('z')),
    #         ('yaw', LaunchConfiguration('yaw'))]
    # )

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
    delayed_local = TimerAction(
            period=10.0, 
            actions=[localization])

    delayed_nav2 = TimerAction(
        period=10.0,  # 3 second delay like your sleep command
        actions=[nav2]
    )
    #undock_and_record = Node(
    #    package='my_robot_bringup',  # Replace with your package name
    #    executable='undock_and_record.py',  # Name of the Python script
    #    output='screen'
    #)
    undock_command = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '/undock', 'irobot_create_msgs/action/Undock', '{}'],
        output='screen'
    )
    # initial_pose_command = ExecuteProcess(
    # cmd=[
    #     'ros2', 'topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
    #     '{'
    #     '"header": {"frame_id": "map"}, '
    #     '"pose": {'
    #     '"pose": {'
    #     '"position": {"x": 0.0, "y": 0.0, "z": 0}, '
    #     '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}'
    #     '}'
    #     '}'
    #     '}'
    # ],
    # output='screen'
    # )
    #
    # # Add after your existing timer actions
    # navigation_goal_command = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'action', 'send_goal', '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose',
    #         '{'
    #         '"pose": {'
    #         '"header": {"frame_id": "map"}, '
    #         '"pose": {'
    #         '"position": {"x": -8.0, "y": 0.0, "z": 0.0}, '  # Set your target coordinates
    #         '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}'
    #         '}'
    #         '}'
    #         '}'
    #     ],
    #     output='screen'
    # )
    #
    undock_with_delay = TimerAction(
        period=25.0,  # Delay in seconds
        actions=[undock_command]
    )
    #
    #
    # init_command = TimerAction(
    #         period=20.0,
    #         actions=[initial_pose_command])
    # goal_command = TimerAction(
    #         period=40.0,
    #         actions=[navigation_goal_command])
    #
    # ign service -s /world/maze/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 1000 --req 'name: "turtlebot4", position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
    # data: true

    #bag_record = ExecuteProcess(
    #cmd=['ros2', 'bag', 'record', '/scan', '/scan_spoofed','/tf', '/tf_static', '/odom', '/cmd_vel'],
    #output='screen'
    #)

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    # ld.add_action(init_command)
    # ld.add_action(undock_with_delay)
    ld.add_action(delayed_local)
    ld.add_action(delayed_nav2)
    # ld.add_action(goal_command)
    return ld


