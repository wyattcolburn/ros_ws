
import os

    <plugin name='ignition::gazebo::systems::Physics' filename='ignition-gazebo-physics-system' />
    <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system' />
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system' />
    <plugin name='ignition::gazebo::systems::Contact' filename='ignition-gazebo-contact-system' />
directory = os.path.expanduser("~/ros_ws/BARN_turtlebot/world_files")

for file in os.listdir(directory):

    raw_file_nam = file.replace(".sdf","")

    file_path = os.path.join(directory, file)
    with open(file_path, 'r') as f:
        content = f.read()

    content = content.replace("<world name='world_name_change.py'>", f"<world name='{raw_file_nam}'>")

    with open(file_path, 'w') as f:
        f.write(content)
