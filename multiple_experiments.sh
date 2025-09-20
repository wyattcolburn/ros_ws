
#!/bin/bash

# Load config.yaml using yq
CSV_FILE=$(yq '.CSV_FILE' config.yaml)

# Customizable params
WORLD_NUMS=($(yq '.WORLD_NUMS[]' config.yaml))
NUM_TRIALS=$(yq '.NUM_TRIALS' config.yaml)
RESET_DELAY=$(yq '.RESET_DELAY' config.yaml)


MODEL_STATIC_DIR="${MODEL_STATIC_DIR:-/home/mobrob/ros_ws/model_path}"

# Read model key or absolute dir from config.yaml
MODEL_KEY=$(yq -r '.MODEL_PATH' config.yaml)   # <-- remove the comma you had
if [[ -z "$MODEL_KEY" || "$MODEL_KEY" == "null" ]]; then
  echo "ERROR: MODEL_PATH missing in config.yaml" >&2
  exit 1
fi

# Resolve to a directory: accept absolute path OR name under created_models/
if [[ -d "$MODEL_KEY" ]]; then
  MODEL_DIR="$MODEL_KEY"
else
  MODEL_DIR="$HOME/ros_ws/created_models/$MODEL_KEY"
fi
if [[ ! -d "$MODEL_DIR" ]]; then
  echo "ERROR: model directory not found: $MODEL_DIR" >&2
  exit 1
fi

# Pick latest .onnx by mtime
LATEST_ONNX=$(ls -1t "$MODEL_DIR"/*.onnx 2>/dev/null | head -n1)
if [[ -z "$LATEST_ONNX" ]]; then
  echo "ERROR: no .onnx found in $MODEL_DIR" >&2
  exit 1
fi
PREFIX="${LATEST_ONNX%.onnx}"

# Prefer strict matches first; fall back to fuzzy mins/maxs
MINS_FILE="${PREFIX}_scaler_mins.txt"
MAXS_FILE="${PREFIX}_scaler_maxs.txt"
[[ -f "$MINS_FILE" ]] || MINS_FILE=$(ls -1t "$MODEL_DIR"/*mins*.txt 2>/dev/null | head -n1 || true)
[[ -f "$MAXS_FILE" ]] || MAXS_FILE=$(ls -1t "$MODEL_DIR"/*maxs*.txt 2>/dev/null | head -n1 || true)

if [[ ! -f "$MINS_FILE" || ! -f "$MAXS_FILE" ]]; then
  echo "ERROR: matching scaler mins/maxs not found for $(basename "$LATEST_ONNX")" >&2
  echo " searched mins: $MINS_FILE  |  maxs: $MAXS_FILE" >&2
  # exit 1
fi

# Stage to versioned folder then atomically update static symlinks
VER_NAME="$(basename "$PREFIX")"                           # e.g. 2025_08_31_14_59
VER_DIR="$MODEL_STATIC_DIR/versions/$VER_NAME"
mkdir -p "$VER_DIR"

cp -f "$LATEST_ONNX" "$VER_DIR/model.onnx"
cp -f "$MINS_FILE"   "$VER_DIR/scaler_mins.txt"
cp -f "$MAXS_FILE"   "$VER_DIR/scaler_maxs.txt"
sync

ln -sfn "$VER_DIR/model.onnx"        "$MODEL_STATIC_DIR/current.onnx"
ln -sfn "$VER_DIR/scaler_mins.txt"   "$MODEL_STATIC_DIR/current_scaler_mins.txt"
ln -sfn "$VER_DIR/scaler_maxs.txt"   "$MODEL_STATIC_DIR/current_scaler_maxs.txt"

echo "Staged model set:"
echo "  $(basename "$LATEST_ONNX")"
echo "  $(basename "$MINS_FILE")"
echo "  $(basename "$MAXS_FILE")"
echo "Static links now point to:"
echo "  $MODEL_STATIC_DIR/current.onnx"
echo "  $MODEL_STATIC_DIR/current_scaler_mins.txt"
echo "  $MODEL_STATIC_DIR/current_scaler_maxs.txt"


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
        TRIAL_RESULT=$(echo "$LAST_LINE" | tr -d '\r\n' | cut -d',' -f9)

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
