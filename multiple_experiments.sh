
#!/bin/bash

# List of world numbers to run experiments on
WORLD_NUMS=(12 36 75 125 203 210 69 187 266)  # <-- add more as needed
# WORLD_NUMS=100
NUM_TRIALS=5
RESET_DELAY=10

# Source environments
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash

# Loop over all world numbers
for WORLD_NUM in "${WORLD_NUMS[@]}"; do
    echo "=== Starting experiments for world $WORLD_NUM ==="

    # Copy the corresponding world/map files
    cp "BARN_turtlebot/world_files/world_${WORLD_NUM}.sdf" "src/turtlebot4_ignition_bringup/worlds/"
    cp "BARN_turtlebot/map_files/yaml_${WORLD_NUM}.yaml" .
    cp "BARN_turtlebot/map_files/map_pgm_${WORLD_NUM}.pgm" .
    export WORLD_NUM=$WORLD_NUM

    for ((trial=1; trial <= NUM_TRIALS; trial++)); do
        echo "___ Starting Trial $trial for world $WORLD_NUM ___"
        echo "Removing robot from simulation..."
        pkill -9 -f "ign gazebo server"
        sleep $RESET_DELAY

        echo "Starting trial $trial with world $WORLD_NUM..."
        ros2 launch my_robot_bringup dockless.launch.py world:=world_${WORLD_NUM} map_file:=yaml_${WORLD_NUM}.yaml

        echo "=== Trial $trial Completed ==="
        LAST_LINE=$(tac ~/ros_ws/baseline_half_radius.csv | grep -m1 .)
        TRIAL_RESULT=$(echo "$LAST_LINE" | tr -d '\r\n' | cut -d',' -f8)

        echo "Last line: '$LAST_LINE'"
        echo "Parsed result: '$TRIAL_RESULT'"

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

    echo "=== Finished all $NUM_TRIALS trials for world $WORLD_NUM ==="
done

echo "___ All experiments completed across ${#WORLD_NUMS[@]} worlds ___"
