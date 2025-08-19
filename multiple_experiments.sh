
#!/bin/bash

# Load config.yaml using yq
CSV_FILE=$(yq '.CSV_FILE' config.yaml)

# Customizable params
WORLD_NUMS=($(yq '.WORLD_NUMS[]' config.yaml))
NUM_TRIALS=$(yq '.NUM_TRIALS' config.yaml)
RESET_DELAY=$(yq '.RESET_DELAY' config.yaml)
# Source environments
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash


# Loop worlds...
for WORLD_NUM in "${WORLD_NUMS[@]}"; do
    echo "=== Starting experiments for world $WORLD_NUM ==="

    cp "BARN_turtlebot/world_files/world_${WORLD_NUM}.sdf" "src/turtlebot4_ignition_bringup/worlds/"
    cp "BARN_turtlebot/map_files/yaml_${WORLD_NUM}.yaml" .
    cp "BARN_turtlebot/map_files/map_pgm_${WORLD_NUM}.pgm" .
    export WORLD_NUM=$WORLD_NUM

    for ((trial=1; trial <= NUM_TRIALS; trial++)); do
        echo "___ Starting Trial $trial for world $WORLD_NUM ___"
        pkill -9 -f "ign gazebo server"
        sleep $RESET_DELAY

        ros2 launch my_robot_bringup dockless.launch.py \
            world:=world_${WORLD_NUM} map_file:=yaml_${WORLD_NUM}.yaml

        LAST_LINE=$(tac "$CSV_FILE" | grep -m1 .)
        TRIAL_RESULT=$(echo "$LAST_LINE" | tr -d '\r\n' | cut -d',' -f8)

        if [[ "$TRIAL_RESULT" == "AMCL TIMEOUT - MAX RETRIES EXCEEDED" ]]; then
            ((trial--))
        elif [[ "$TRIAL_RESULT" == "NAV_SERVER_UNAVAILABLE" ]]; then
            ((trial--))
        else
            echo "Trial $trial completed with result: $TRIAL_RESULT"
        fi
        sleep 2
    done
done
