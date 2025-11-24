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
    DeclareLaunchArgument('world_num', default_value='0',
                          description='What trial?'),
    DeclareLaunchArgument('auto_start_nav', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically start navigation sequence'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

def load_barn_path(world_num, resample_step=0.20, smooth_window=5):
    import numpy as np
    import math
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped

    def arc_lengths(xy):
        d = np.diff(xy, axis=0)
        seg = np.hypot(d[:, 0], d[:, 1])
        s = np.concatenate([[0.0], np.cumsum(seg)])
        return s, seg

    def resample_by_arclen(xy, step=0.12):
        keep = [0]
        for i in range(1, len(xy)):
            if not np.allclose(xy[i], xy[keep[-1]], atol=1e-8):
                keep.append(i)
        xy = xy[keep]
        if len(xy) < 2:
            return xy
        s, _ = arc_lengths(xy)
        if s[-1] < step:
            return xy
        s_new = np.arange(0.0, s[-1] + 1e-6, step)
        x = np.interp(s_new, s, xy[:, 0])
        y = np.interp(s_new, s, xy[:, 1])
        return np.stack([x, y], axis=1)

    def smooth_xy(xy, window=5):
        if window < 3 or window % 2 == 0 or len(xy) < window:
            return xy
        k = window // 2
        pad = np.pad(xy, ((k, k), (0, 0)), mode='edge')
        kern = np.ones((window, 1)) / window
        xs = np.convolve(pad[:, 0], kern[:, 0], mode='valid')
        ys = np.convolve(pad[:, 1], kern[:, 0], mode='valid')
        return np.stack([xs, ys], axis=1)

    def yaw_from_diffs(dx, dy, last_yaw=None):
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return last_yaw if last_yaw is not None else 0.0
        return math.atan2(dy, dx)

    def yaw_to_quat(yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return 0.0, 0.0, qz, qw

    barn_path = np.load(
        os.path.expanduser(f'~/ros_ws/BARN_turtlebot/path_files/path_{world_num}.npy')
    ).astype(float)


    xy = np.array([path_coord_to_gazebo_coord(x, y) for x, y in barn_path], dtype=float)
    xy = resample_by_arclen(xy, step=resample_step)
    xy = smooth_xy(xy, window=smooth_window)

    yaws = []
    last_yaw = None
    for i in range(len(xy)):
        if i == len(xy) - 1:
            yaw = last_yaw if last_yaw is not None else 0.0
        else:
            dx = xy[i + 1, 0] - xy[i, 0]
            dy = xy[i + 1, 1] - xy[i, 1]
            yaw = yaw_from_diffs(dx, dy, last_yaw)
        yaws.append(yaw)
        last_yaw = yaw

    return xy


def path_coord_to_gazebo_coord(x, y):
    # Tis is from the jackal_timer github repo from dperille (UT-AUSTIN LAB)
    RADIUS = .075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)


def yaw_calculation(x1, y1, x2, y2):

    return math.atan2(y2-y1, x2-x1)


def generate_launch_description():

    world_num = int(os.environ.get('WORLD_NUM', '0'))
    path = np.load(os.path.expanduser(
        f'~/ros_ws/BARN_turtlebot/path_files/path_{world_num}.npy'))

    preprocessed_path = load_barn_path(world_num)
    first_location_gazebo= (preprocessed_path[0][0], preprocessed_path[0][1])
    second_location_gazebo= (preprocessed_path[1][0], preprocessed_path[1][1])
    # second_location_gazebo = path_coord_to_gazebo_coord(path[1][0], path[1][1])
    # third_location_gazebo = path_coord_to_gazebo_coord(path[3][0], path[3][1])
    # fourth_location_gazebo = path_coord_to_gazebo_coord(path[4][0], path[4][1])
    # yaw = yaw_calculation(start_location_gazebo[0], start_location_gazebo[1], second_location_gazebo[0], second_location_gazebo[1])
    yaw = yaw_calculation(first_location_gazebo[0], first_location_gazebo[1], second_location_gazebo[0], second_location_gazebo[1])

    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    # Paths
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
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
            ('x', f'{first_location_gazebo[0]}'),
            ('y', f'{first_location_gazebo[1]}'),
            ('z', LaunchConfiguration('z')),
            ('yaw', f'{yaw}')]
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
        executable='barn_nodock',
        name='one_shot_trial',
        output='screen',
        parameters=[{
            'initial_x':  float(first_location_gazebo[0]),
            'initial_y': float(first_location_gazebo[1]),
            'initial_yaw': float(yaw),
            'wait_after_undock': 2.0,
            'pose_init_delay': 1.0,
            'world_num': world_num,
        }]
    )
    # Add delay for nav2 to start after localization

    delayed_local = TimerAction(
        period=10.0,
        actions=[localization])

    delayed_nav2 = TimerAction(
        period=10.0,  # 3 second delay like your sleep command
        actions=[nav2]
    )
    delayed_barn = TimerAction(
        period=35.0,  # 3 second delay like your sleep command
        actions=[barn_one_shot]
    )
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
    # ld.add_action(undock_with_delay)
    ld.add_action(delayed_local)
    ld.add_action(delayed_nav2)
    ld.add_action(delayed_barn)
    return ld
