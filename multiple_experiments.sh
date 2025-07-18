#!/bin/bash
NUM_TRIALS=1
RESET_DELAY=10
WORLD_NUM=0  # Add this - specify which world to use
WORLD="barn_world_${WORLD_NUM}"  # Make world name dynamic

export WORLD_NUM=$WORLD_NUM
echo "Starting multi-trial experiment with $NUM_TRIALS trials using world $WORLD_NUM"

for ((trial=1; trial <= NUM_TRIALS; trial++)); do
    echo "___ Starting Trial $trial with world $WORLD_NUM ___"
    echo "Removing robot from simulation..."
    pkill -f "ign gazebo"
    sleep $RESET_DELAY
    
    # Start the trial - pass world_num as argument
    echo "Starting trial $trial with world $WORLD_NUM..."
    ros2 launch my_robot_bringup barn_test.launch.py world:="BARN_turtlebot/world_files/world_0" map:="BARN_turtlebot/map_files/yaml_0"

    echo "=== Trial $trial Completed ==="
    sleep 2
done
echo "___ All $NUM_TRIALS completed __"
