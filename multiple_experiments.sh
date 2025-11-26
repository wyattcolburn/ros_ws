#!/bin/bash
set -eE -o pipefail   # no -uAMCL
source ~/.bashrc
source /opt/ros/humble/setup.bash
source "$HOME/ros_ws/install/setup.bash" || true


CFG="config.yaml"
ROOT="$HOME/ros_ws"
MODEL_STATIC_DIR="${MODEL_STATIC_DIR:-$ROOT/model_path}"

# ---- Read config ----
mapfile -t WORLD_NUMS < <(yq -r '.WORLD_NUMS[]' "$CFG")
NUM_TRIALS=$(yq -r '.NUM_TRIALS' "$CFG")
RESET_DELAY=$(yq -r '.RESET_DELAY' "$CFG")

# Prefer lists; fall back to single values if present
if yq -e '.MODEL_PATHS' "$CFG" >/dev/null 2>&1; then
  mapfile -t MODEL_PATHS < <(yq -r '.MODEL_PATHS[]' "$CFG")
else
  MODEL_PATHS=($(yq -r '.MODEL_PATH // ""' "$CFG"))
fi

if yq -e '.CSV_FILES' "$CFG" >/dev/null 2>&1; then
  mapfile -t CSV_FILES < <(yq -r '.CSV_FILES[]' "$CFG")
else
  CSV_FILES=($(yq -r '.CSV_FILE // "timer_test/baseline.csv"' "$CFG"))
fi

# If only one CSV provided, reuse it for all models
if [[ ${#CSV_FILES[@]} -lt ${#MODEL_PATHS[@]} ]]; then
  base_csv="${CSV_FILES[0]}"
  CSV_FILES=()
  for mp in "${MODEL_PATHS[@]}"; do
    # derive a unique CSV per model if needed
    name="$(basename "$mp")"
    CSV_FILES+=("timer_test/${name}.csv")
  done
fi

# ---- envs ----
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"

# ---- Loop over models ----
for mi in "${!MODEL_PATHS[@]}"; do
  MODEL_KEY="${MODEL_PATHS[$mi]}"
  CSV_REL="${CSV_FILES[$mi]}"
  CSV_ABS="$ROOT/$CSV_REL"
  mkdir -p "$(dirname "$CSV_ABS")"

  echo "=== MODEL[$mi]: $MODEL_KEY  -> CSV: $CSV_REL ==="

  # Resolve model directory (absolute path accepted; else under created_models/)
  if [[ -d "$MODEL_KEY" ]]; then
    MODEL_DIR="$MODEL_KEY"
  else
    MODEL_DIR="$ROOT/$MODEL_KEY"
    [[ -d "$MODEL_DIR" ]] || MODEL_DIR="$ROOT/created_models/$MODEL_KEY"
  fi
  if [[ ! -d "$MODEL_DIR" ]]; then
    echo "ERROR: model directory not found: $MODEL_KEY (resolved: $MODEL_DIR)" >&2
    exit 1
  fi

  # Pick latest ONNX + scaler files
  LATEST_ONNX=$(ls -1t "$MODEL_DIR"/*.onnx 2>/dev/null | head -n1 || true)
  if [[ -z "$LATEST_ONNX" ]]; then
    echo "ERROR: no .onnx found in $MODEL_DIR" >&2
    exit 1
  fi
  PREFIX="${LATEST_ONNX%.onnx}"

  MINS_FILE="${PREFIX}_scaler_mins.txt"
  MAXS_FILE="${PREFIX}_scaler_maxs.txt"
  [[ -f "$MINS_FILE" ]] || MINS_FILE=$(ls -1t "$MODEL_DIR"/*mins*.txt 2>/dev/null | head -n1 || true)
  [[ -f "$MAXS_FILE" ]] || MAXS_FILE=$(ls -1t "$MODEL_DIR"/*maxs*.txt 2>/dev/null | head -n1 || true)
  export CONTROLLER_VARIANT=MLP

    colcon build --packages-select onnx --cmake-clean-cache
    source "$HOME/ros_ws/install/setup.bash"
  if [[ ! -f "$MINS_FILE" || ! -f "$MAXS_FILE" ]]; then
    echo "WARN: scaler mins/maxs not found matching $(basename "$LATEST_ONNX")"
    echo "      mins: $MINS_FILE"
    echo "      maxs: $MAXS_FILE"
    export CONTROLLER_VARIANT=CNN
    colcon build --packages-select onnx --cmake-clean-cache
    source "$HOME/ros_ws/install/setup.bash"
    # continue without exiting if that's acceptable for your controller
  fi

  # Stage to versioned folder then update static symlinks
  VER_NAME="$(basename "$PREFIX")"
  VER_DIR="$MODEL_STATIC_DIR/versions/$VER_NAME"
  mkdir -p "$VER_DIR"
  cp -f "$LATEST_ONNX" "$VER_DIR/model.onnx"
  [[ -f "$MINS_FILE" ]] && cp -f "$MINS_FILE" "$VER_DIR/scaler_mins.txt"
  [[ -f "$MAXS_FILE" ]] && cp -f "$MAXS_FILE" "$VER_DIR/scaler_maxs.txt"
  sync

  ln -sfn "$VER_DIR/model.onnx"             "$MODEL_STATIC_DIR/current.onnx"
  [[ -f "$VER_DIR/scaler_mins.txt" ]] && ln -sfn "$VER_DIR/scaler_mins.txt"  "$MODEL_STATIC_DIR/current_scaler_mins.txt"
  [[ -f "$VER_DIR/scaler_maxs.txt" ]] && ln -sfn "$VER_DIR/scaler_maxs.txt"  "$MODEL_STATIC_DIR/current_scaler_maxs.txt"

  echo "Staged model:"
  echo "  $(basename "$LATEST_ONNX")"
  [[ -f "$VER_DIR/scaler_mins.txt" ]] && echo "  $(basename "$VER_DIR/scaler_mins.txt")"
  [[ -f "$VER_DIR/scaler_maxs.txt" ]] && echo "  $(basename "$VER_DIR/scaler_maxs.txt")"

  # Export index so the node selects MODEL_PATHS[idx], CSV_FILES[idx]
  export MODEL_INDEX="$mi"
  # (Optional) explicitly point the node at this config file (keeps things robust if cwd changes)
  export CONFIG_YAML="$ROOT/config.yaml"

  # ---- Loop worlds and trials for this model ----
  for WORLD_NUM in "${WORLD_NUMS[@]}"; do
    echo "=== Starting experiments for world $WORLD_NUM (model idx $mi) ==="

    cp "BARN_turtlebot/world_files/world_${WORLD_NUM}.sdf" "src/turtlebot4_ignition_bringup/worlds/"
    cp "BARN_turtlebot/map_files/yaml_${WORLD_NUM}.yaml" .
    cp "BARN_turtlebot/map_files/map_pgm_${WORLD_NUM}.pgm" .
    export WORLD_NUM

    for ((trial=1; trial <= NUM_TRIALS; trial++)); do
      echo "___ Starting Trial $trial for world $WORLD_NUM (model idx $mi) ___"
      pkill -9 -f "ign gazebo server" || true
      sleep "$RESET_DELAY"

      # Launch and block until exit
      ros2 launch my_robot_bringup dwa_baseline.launch.py \
        world:=world_${WORLD_NUM} map_file:=yaml_${WORLD_NUM}.yaml

      # Read last line from this model's CSV
      if [[ -f "$CSV_ABS" ]]; then
        LAST_LINE=$(tac "$CSV_ABS" | grep -m1 . || true)
      else
        echo "WARN: CSV not found yet: $CSV_ABS"
        LAST_LINE=""
      fi
      TRIAL_RESULT=$(echo "$LAST_LINE" | tr -d '\r\n' | cut -d',' -f9 || true)

      if [[ "$TRIAL_RESULT" == "AMCL TIMEOUT - MAX RETRIES EXCEEDED" ]]; then
        ((trial--))
      elif [[ "$TRIAL_RESULT" == "NAV_SERVER_UNAVAILABLE" ]]; then
        ((trial--))
      else
        echo "Trial $trial completed with result: ${TRIAL_RESULT:-UNKNOWN}"
      fi
      sleep 2
    done
  done
done
