#!/bin/bash
# Turtlebot4 Simulation and Navigation Launcher Script with multiple terminals

# Function to check if gnome-terminal is available, otherwise try xterm
launch_terminal() {
    local command="$1"
    local title="$2"
    
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --title="$title" -- bash -c "$command; exec bash"
    elif command -v xterm &> /dev/null; then
        xterm -T "$title" -e "bash -c '$command; exec bash'" &
    else
        echo "Error: Neither gnome-terminal nor xterm is available"
        exit 1
    fi
}

# Launch the simulator in a new terminal
echo "Starting Turtlebot4 Ignition simulator..."
launch_terminal "ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py" "TB4 Simulator"

# Wait for simulator to initialize
sleep 5

# Launch localization in a new terminal
echo "Starting localization with custom map..."
launch_terminal "ros2 launch turtlebot4_navigation localization.launch.py map:=/home/wyatt/ros_ws/big_map_april_4.yaml use_sim_time:=true" "TB4 Localization"

# Wait for localization to initialize
sleep 3

# Launch navigation stack in a new terminal
echo "Starting navigation stack..."
launch_terminal "ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true" "TB4 Navigation"

echo "All processes have been launched in separate terminals."
