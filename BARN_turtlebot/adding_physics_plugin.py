import sys
import os

# Plugins to ensure are present
PLUGIN_LINES = [
    "    <plugin name='ignition::gazebo::systems::Physics' filename='ignition-gazebo-physics-system' />\n",
    "    <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system' />\n",
    "    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system' />\n",
    "    <plugin name='ignition::gazebo::systems::Contact' filename='ignition-gazebo-contact-system' />\n"
]

def ensure_plugins_in_file(filepath):
    with open(filepath, "r") as f:
        lines = f.readlines()

    # Ensure at least 7 lines to safely check
    while len(lines) < 7:
        lines.append("\n")

    # Get lines 4–7 (0-indexed -> 3:7)
    snippet = "".join(lines[2:6])

    # Check if the first plugin line is missing (indicator of all missing)
    if PLUGIN_LINES[0].strip() not in snippet:
        print(f"Updating: {filepath}")
        # Insert after line 3 (i.e., before line 4)
        new_lines = lines[:3] + PLUGIN_LINES + lines[3:]
        with open(filepath, "w") as f:
            f.writelines(new_lines)
    else:
        print(f"Skipping (plugins present): {filepath}")

def process_directory(root_dir):
    for root, _, files in os.walk(root_dir):
        for filename in files:
            filepath = os.path.join(root, filename)
            ensure_plugins_in_file(filepath)

if __name__ == "__main__":
    directory = input("Enter the directory path: ").strip()
    process_directory(directory)
