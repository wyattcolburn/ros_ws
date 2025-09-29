
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
import os, yaml
ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='turtle', description='Robot namespace (no leading slash)'),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true','false'], description='Start RViz'),
    DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'ros_ws', 'hallway_1.yaml']),
        description='Map YAML for localization'
    ),
    DeclareLaunchArgument('auto_start_nav', default_value='true', choices=['true','false'], description='Autostart Nav2 lifecycle'),
]

def yaml_reader():
    """Read the configs from config.yaml"""
    filepath = os.path.join(os.path.expanduser(
        '~'), 'ros_ws', 'hardware.yaml')  # Fixed typo: trail -> trial
    with open(filepath, "r") as file:
        config = yaml.safe_load(file)

        # Example access
        init_x = config["INIT_X"]
        init_y = config["INIT_Y"]
        init_yaw = config["INIT_YAW"]
        goal_x = config["GOAL_X"]
        goal_y = config["GOAL_Y"]
        goal_yaw = config["GOAL_YAW"]
        return (init_x, init_y, init_yaw, goal_x, goal_y, goal_yaw)
def generate_launch_description():

    init_x, init_y, init_yaw, goal_x, goal_y, goal_yaw = yaml_reader()
    ns = LaunchConfiguration('namespace')

    pkg_nav = get_package_share_directory('turtlebot4_navigation')
    pkg_viz = get_package_share_directory('turtlebot4_viz')

    loc_launch = PathJoinSubstitution([pkg_nav, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution([pkg_nav, 'launch', 'nav2.launch.py'])
    rviz_launch = PathJoinSubstitution([pkg_viz, 'launch', 'view_robot.launch.py'])

    # Optional RViz (namespaced)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments={'namespace': ns}.items(),
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Localization (real hardware: use_sim_time=false)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([loc_launch]),
        launch_arguments={
            'namespace': ns,
            'map': LaunchConfiguration('map_file'),
            'use_sim_time': 'false'
        }.items()
    )

    # Nav2 (autostart wired to your arg; add params_file here if needed)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': 'false',
            'autostart': LaunchConfiguration('auto_start_nav'),
            # 'params_file': '/path/to/nav2_params_hw.yaml',
        }.items()
    )

    # Your extra nodes (now namespaced)
    publish_features_node = Node(
        package='publish_features',
        executable='publish_features_node',
        name='publish_features',
        namespace=ns,
        output='log',
        parameters=[],
    )

    middle_man_node = Node(
        package='middle_man',
        executable='middle_man_barn',
        name='middle_man',
        namespace=ns,
        output='screen',
        parameters=[{'obstacle_config': '/home/mobrob/ros_ws/config.yaml'}],
    )
    hardware_node = Node(
        package='my_robot_bringup',
        executable='hardware',
        name='hardware_test',
        output='screen',
        namespace=ns,
        parameters=[{
            'initial_x': init_x,
            'initial_y': init_y,
            'initial_yaw': init_yaw,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
            'wait_after_undock': 2.0,
            'pose_init_delay': 1.0,
        }]
    )

    # Build LD in your style
    ld = LaunchDescription(ARGUMENTS)

    # Order: RViz immediately (if enabled), then localization, then Nav2, then your nodes
    ld.add_action(TimerAction(period=0.0, actions=[rviz]))
    ld.add_action(TimerAction(period=0.0, actions=[localization]))
    ld.add_action(TimerAction(period=3.0, actions=[nav2]))
    # ld.add_action(TimerAction(period=5.0, actions=[hardware_node]))
    # ld.add_action(TimerAction(period=6.0, actions=[publish_features_node]))
    # ld.add_action(TimerAction(period=8.0, actions=[middle_man_node]))

    return ld
