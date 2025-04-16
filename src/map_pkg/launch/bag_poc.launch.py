import launch

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/scan', '/tf', '/tf_static'],
            output='screen'
        )
    ])

