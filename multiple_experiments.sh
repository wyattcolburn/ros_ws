#!/bin/bash

NUM_TRIALS=3
RESET_DELAY=10
WORLD="EXP1"

echo "Starting multi-trial experiment with $NUM_TRIALS trials"

for ((trial=1; trial <= NUM_TRIALS; trial++)); do
    echo "___ Starting Trial $trial ___"

    echo "Removing robot from simulation..."
    pkill -f "ign gazebo"
    sleep $RESET_DELAY
    
    # Start the trial
    echo "Starting trial $trial..."
    ros2 launch my_robot_bringup one_shot_launch.launch.py
    

    echo "=== Trial $trial Completed ==="
    
    sleep 2

done

echo "___ All $NUM_TRIALS completed __" 
