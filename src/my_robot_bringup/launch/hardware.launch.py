
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node

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

def generate_launch_description():
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

    # Build LD in your style
    ld = LaunchDescription(ARGUMENTS)

    # Order: RViz immediately (if enabled), then localization, then Nav2, then your nodes
    ld.add_action(TimerAction(period=0.0, actions=[rviz]))
    ld.add_action(TimerAction(period=0.0, actions=[localization]))
    ld.add_action(TimerAction(period=3.0, actions=[nav2]))
    # ld.add_action(TimerAction(period=6.0, actions=[publish_features_node]))
    # ld.add_action(TimerAction(period=8.0, actions=[middle_man_node]))

    return ld
