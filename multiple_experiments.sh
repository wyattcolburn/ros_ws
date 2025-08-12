#!/bin/bash
NUM_TRIALS=10
RESET_DELAY=10
WORLD_NUM=150  # Add this - specify which world to use
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
cp "BARN_turtlebot/world_files/world_${WORLD_NUM}.sdf" "src/turtlebot4_ignition_bringup/worlds/"
cp "BARN_turtlebot/map_files/yaml_${WORLD_NUM}.yaml" .
cp "BARN_turtlebot/map_files/map_pgm_${WORLD_NUM}.pgm" .
export WORLD_NUM=$WORLD_NUM
echo "Starting multi-trial experiment with $NUM_TRIALS trials using world $WORLD_NUM"

for ((trial=1; trial <= NUM_TRIALS; trial++)); do
    echo "___ Starting Trial $trial with world $WORLD_NUM ___"
    echo "Removing robot from simulation..."
    pkill -9 -f "ign gazebo server"
    sleep $RESET_DELAY
    
    # Start the trial - pass world_num as argument
    echo "Starting trial $trial with world $WORLD_NUM..."
    ros2 launch my_robot_bringup dockless.launch.py world:=world_${WORLD_NUM} map_file:=yaml_${WORLD_NUM}.yaml

    echo "=== Trial $trial Completed ==="
    LAST_LINE=$(tail -n 1 ~/ros_ws/trial_results_world_num.csv)
    TRIAL_RESULT=$(echo "$LAST_LINE" | cut -d',' -f8)

    if [[ "$TRIAL_RESULT" == "AMCL TIMEOUT - MAX RETRIES EXCEEDED" ]]; then
        echo "Skipping Trial $trial due to AMCL timeout (retrying same trial number)"
        ((trial--))
    elif [[ "$TRIAL_RESULT" == "NAV_SERVER_UNAVAILABLE" ]]; then
        echo "Skipping Trial $trial due to NAV_SERVER error (retrying same trial number)"
        ((trial--))
    else
        echo "Trial $trial completed with result: $TRIAL_RESULT"
    fi  
    sleep 2
done
echo "___ All $NUM_TRIALS completed __"
