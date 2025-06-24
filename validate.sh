#!/bin/bash
# Turtlebot4 Simulation and Navigation Launcher Script using tmux

SESSION_NAME="tb4_sim"
echo "Building workspace (skipping prep package)..."
cd ~/ros_ws
colcon build --packages-skip prep
source install/setup.bash
sleep 10
# Kill existing tmux session if it exists
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Killing existing tmux session: $SESSION_NAME"
    tmux kill-session -t $SESSION_NAME
fi

tmux new-session -d -s $SESSION_NAME -n rviz "rviz2 -d validation.rviz"
sleep 3
# Start new tmux session
tmux new-window -t $SESSION_NAME -n sim "ros2 launch my_robot_bringup exp_validation.launch.py"

# Give simulator time to start
sleep 5

# New window for localization
tmux new-window -t $SESSION_NAME -n localization "ros2 launch turtlebot4_navigation localization.launch.py map:=one_obstacle.yaml use_sim_time:=true"

# Give localization time to start
sleep 3

# New window for nav2
tmux new-window -t $SESSION_NAME -n nav2 "ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true"
#
# sleep 10
tmux new-window -t $SESSION_NAME -n publish_features "colcon build --packages-select publish_features && source install/setup.bash && ros2 run publish_features publish_features_node"

tmux new-window -t $SESSION_NAME -n middle_man "colcon build --packages-select middle_man && source install/setup.bash && ros2 run middle_man middle_man_valid"
# Attach to session
tmux attach-session -t $SESSION_NAME

