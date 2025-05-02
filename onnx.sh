#!/bin/bash
# Turtlebot4 Simulation and Navigation Launcher Script using tmux

SESSION_NAME="tb4_sim"

# Kill existing tmux session if it exists
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Killing existing tmux session: $SESSION_NAME"
    tmux kill-session -t $SESSION_NAME
fi

# Start new tmux session
tmux new-session -d -s $SESSION_NAME -n sim "ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py"

# Give simulator time to start
sleep 5

# New window for localization
tmux new-window -t $SESSION_NAME:1 -n localization "ros2 launch turtlebot4_navigation localization.launch.py map:=/home/wyattcolburn/ros_ws/big_map_april_4.yaml use_sim_time:=true"

# Give localization time to start
sleep 3

# New window for nav2
tmux new-window -t $SESSION_NAME:2 -n nav2 "ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true"

# Attach to session
tmux attach-session -t $SESSION_NAME

