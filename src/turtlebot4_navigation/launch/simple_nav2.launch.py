
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'], description='Use sim time'),
    DeclareLaunchArgument('params_file',
                          default_value=PathJoinSubstitution([
                              get_package_share_directory('turtlebot4_navigation'),
                              'config', 'nav2_copy.yaml']),
                          description='Nav2 parameters'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('bt_xml',
                          default_value=PathJoinSubstitution([
                              get_package_share_directory('turtlebot4_navigation'),
                              'launch', 'plannerless_follow_path.xml']),
                          description='Planner-free BT XML'),
]

def launch_setup(context, *args, **kwargs):
    nav2_params  = LaunchConfiguration('params_file')
    namespace    = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_xml       = LaunchConfiguration('bt_xml')

    ns_str = namespace.perform(context)
    if ns_str and not ns_str.startswith('/'):
        ns_str = '/' + ns_str

    # --- Nodes for the navigation stack only (NO map_server / amcl here) ---
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, nav2_params]
    )


    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', namespace=namespace, output='screen',
        parameters=[{'use_sim_time': use_sim_time}, nav2_params]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', namespace=namespace, output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'default_nav_to_pose_bt_xml': bt_xml,
             'default_nav_through_poses_bt_xml': bt_xml},
            nav2_params
        ]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',  # unique name
        namespace=namespace, output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'local_costmap',
                'controller_server',
                'bt_navigator'
            ]
        }]
    )

    nav2_group = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(ns_str + '/local_costmap/scan', ns_str + '/scan'),
        local_costmap,
        controller_server,
        bt_navigator,
        lifecycle_manager_navigation,
    ])

    return [nav2_group]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
