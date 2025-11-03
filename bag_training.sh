
#!/bin/bash
# Turtlebot4 Simulation and Navigation Launcher Script using tmux
# The point of this script is to call the simulator and run multiple_seg.py to 
# convert rosbag data from random walk and convert to training.csv
SESSION_NAME="bag_training"
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
tmux new-window -t $SESSION_NAME -n sim "ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py"

# Give simulator time to start
sleep 5

# New window for localization
tmux new-window -t $SESSION_NAME -n localization "ros2 launch turtlebot4_navigation localization.launch.py map:=big_map_april_4.yaml use_sim_time:=true"

# Give localization time to start
sleep 3

# New window for nav2

tmux new-window -t $SESSION_NAME -n middle_man "colcon build --packages-select middle_man && source install/setup.bash && ros2 run middle_man middle_man_valid"
# Attach to session
tmux attach-session -t $SESSION_NAME

