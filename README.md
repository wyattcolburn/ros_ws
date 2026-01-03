# LFH Thesis: Neural Network Navigation Controller

> Replacing DWA local controller with a neural network trained on hallucinated LiDAR data for autonomous robot navigation

## 🛠️ Tech Stack

- **ROS 2 Humble** - Robot Operating System
- **C++** - Controller implementation
- **Python** - Data processing and training
- **Keras/TensorFlow** - Neural network training
- **ONNX** - Model deployment
- **Gazebo Ignition** - Simulation environment
- 
Check out the [model repository](https://github.com/wyattcolburn/model) for more details on how to train the models

Good luck. A lot of the work has been done already, just look for it. Contact me @wyattdcolburn@gmail.com for any assistance
---

## 🔧 Setup Modifications

### Removed Dock Spawning

The default TurtleBot4 simulation spawns with a charging dock. This has been removed as it's unnecessary for navigation experiments.

**Modified file:** `turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py`

<details>
<summary>View changes</summary>
```bash
diff src/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py \
     /opt/ros/humble/share/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py
```

**Changes:**
- Removed dock description import and launch inclusion
- Removed dock spawn node
- Removed dock positioning calculations
- Removed `dock_name` parameters from ROS-Ignition bridge

</details>

---

# Dataset Generation and Model Training Pipeline

This guide walks through generating training data from random walks, training a neural network controller, and evaluating its performance.

---

## 1. Configure Random Walk Policy

Define which random walk behavior to use in your launch file:

**File:** `training.launch.py` (line 63)
```python
random_walk_node = TimerAction(
    period=15.0,
    actions=[Node(
        package='my_robot_bringup',
        executable='gaussian_random_walk',  # ← Choose your policy here
        output='screen')]
)
```

Available policies:
- `gaussian_random_walk` - Gaussian-based exploration
- `uniform_random_walk` - Uniform random movements
- (Add your custom policies here)

---

## 2. Set Output Directory

Edit `config.yaml` to specify where raw bag files will be stored:
```yaml
RANDOM_WALK_BAG_DKR: readme_example
```

This creates bags in: `~/ros_ws/ros_bag/readme_example/`

---

## 3. Generate Random Walk Data

Build and launch the random walk data collection:
```bash
# Build the package
cd ~/ros_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash

# Start data collection
ros2 launch my_robot_bringup training.launch.py
```

The robot will spawn and begin executing random walks. Each run creates a timestamped bag file in your configured directory.

**Monitor progress:** The robot will automatically record `/scan`, `/odom`, `/cmd_vel`, and transform data.

---

## 4. Verify Data Collection

Count odometry messages in your collected bags:
```bash
python3 odom_counter.py ros_bag/readme_example
```

**Expected output:**
```
Processing bags in ros_bag/readme_example...
  2026-01-01_12-40-05_gaus: 7,542 odometry messages
  2026-01-01_13-06-59_gaus: 8,231 odometry messages
...
Total: 45,128 odometry messages across 6 bags
```

---

## 5. Generate Training Dataset

Convert raw bag files into training-ready CSV files with hallucinated obstacles.

### a) Configure Dataset Generation

Edit `config.yaml`:
```yaml
TRAINING_DKR: readme_example  # Must match RANDOM_WALK_BAG_DKR
ASYM_FLAG: 1                  # 1 = asymmetric obstacles, 0 = symmetric
OFFSET: 0.5                   # Corridor half-width (meters)
RADIUS: 0.15                  # Obstacle radius (meters)
```

### b) Start Required ROS Nodes

Open **three separate terminals**:

**Terminal 1:** Launch simulator and localization
```bash
ros2 launch my_robot_bringup simple_bringup.launch.py
```

**Terminal 2:** Start RViz for monitoring
```bash
rviz2
```

**Terminal 3:** Run dataset generation script
```bash
./training_script # Or run the appropriate command
```

### c) Set Initial Pose in RViz

1. In RViz, click **"2D Pose Estimate"**
2. Click on the map where the robot should start
3. The script will automatically begin processing all bags

**Output:**
- Creates `input_data/` folders for each bag segment
- Generates `odom_data.csv`, `cmd_vel_output.csv`, `lidar_data.csv`, `local_goals.csv`
- Saves visualization frames in `frames/` directories

---

## 6. Train Neural Network

Train your navigation controller using the generated dataset:
```bash
cd ~/ros_ws/model_repo
python3 neural_net.py ~/ros_ws/ros_bag/readme_example --single --large_dkr
```

**Arguments:**
- `--single` - Train a single model (vs. ensemble)
- `--large_dkr` - Use all data in directory (vs. subset)

**Training output:**
```
Found 9 segments with 45,128 total data points
Training MLP model...
Epoch 1/150: loss: 0.0234 - val_loss: 0.0189
...
Model saved to: readme_example_mlp_model.h5
ONNX export: readme_example_mlp_model.onnx
```

---

## 7. Deploy Model

Move the trained model to the deployment directory:
```bash
# Create model directory
mkdir -p ~/ros_ws/created_models/readme_example_model

# Move model files
mv readme_example_mlp_model.onnx ~/ros_ws/created_models/readme_example_model/
mv readme_example_mlp_model.h5 ~/ros_ws/created_models/readme_example_model/
```

---

## 8. Configure Testing

Edit `config.yaml` to point to your new model:
```yaml
# Model configuration
MODEL_PATH: created_models/readme_example_model/readme_example_mlp_model.onnx
ASYM_FLAG: 1

# Testing parameters
NUM_TRIALS: 10
MAX_TRIAL_TIME: 120.0
```

---

## 9. Run Experiments

Execute the batch testing script:
```bash
./multiple_scripts.sh
```

This will:
- Run your model across multiple test worlds
- Record success/failure rates
- Log kinematics and navigation metrics
- Save results to CSV files

**Progress output:**
```
Testing world_12 (trial 1/10)...
  ✓ SUCCESS - Reached goal in 34.2s
Testing world_12 (trial 2/10)...
  ✓ SUCCESS - Reached goal in 31.8s
...
```

---

## 10. Analyze Results

Generate summary statistics and visualizations:
```bash
python3 trial_summary.py path/to/results_directory
```

**Outputs:**
- `successes_per_world.png` - Bar chart of success rates
- `success_vs_failure_per_world.png` - Stacked success/failure counts  
- `avg_fraction_per_world.png` - Progress toward goal per world
- `success_kinematics_per_world.csv` - Velocity statistics for successful runs

**Example summary:**
```
World 12:  8/10 successes (80%), avg progress: 0.92
World 36:  9/10 successes (90%), avg progress: 0.95
World 75:  6/10 successes (60%), avg progress: 0.78
...
Overall: 67/90 successes (74.4%)
```

---

## Troubleshooting

**Problem:** "No bags found in directory"
- **Solution:** Check that `RANDOM_WALK_BAG_DKR` matches your actual bag directory name

**Problem:** "NaN loss during training"
- **Solution:** Verify LiDAR data contains valid ranges (use `view` tool to inspect CSV files)

**Problem:** "TF transform not available"
- **Solution:** Ensure `simple_bringup.launch.py` is running before dataset generation

**Problem:** Model performs poorly
- **Solution:** Collect more data (aim for 50k+ odometry points), verify OFFSET/RADIUS match your environment

---

## Directory Structure

After completing this pipeline:
```
~/ros_ws/
├── ros_bag/
│   └── readme_example/
│       ├── 2026-01-01_12-40-05_gaus/
│       │   ├── input_data/
│       │   │   ├── odom_data.csv
│       │   │   ├── cmd_vel_output.csv
│       │   │   ├── lidar_data.csv
│       │   │   └── local_goals.csv
│       │   └── seg_0/, seg_1/, ...
│       └── config_meta_data.yaml
├── created_models/
│   └── readme_example_model/
│       ├── readme_example_mlp_model.onnx
│       └── readme_example_mlp_model.h5
└── results/
    └── readme_example_results/
        ├── baseline.csv
        ├── successes_per_world.png
        └── success_kinematics_per_world.csv
```

## World File Configuration

### CRITICAL: Collision Detection Setup

The BARN dataset's original `.sdf` world files are **missing required Gazebo plugins** for collision detection. You must add these plugins to each world file before use. 

**Required plugins** (add immediately after `<world>` tag, before any other elements):
```xml
<sdf version='1.6'>
  <world name='world_0'>
    <!-- REQUIRED: Add these plugin lines HERE (before <light>) -->
    <plugin name='ignition::gazebo::systems::Physics' filename='ignition-gazebo-physics-system' />
    <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system' />
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system' />
    <plugin name='ignition::gazebo::systems::Contact' filename='ignition-gazebo-contact-system' />
    
    <!-- Existing lighting configuration (comes AFTER plugins) -->
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>
    
    <!-- Rest of world definition... -->
  </world>
</sdf>
```

### What These Plugins Do

| Plugin | Purpose |
|--------|---------|
| `Physics` | Enables physics simulation and collision detection |
| `UserCommands` | Allows spawning/removing entities during runtime |
| `SceneBroadcaster` | Publishes scene state to ROS topics |
| `Contact` | **Critical:** Detects robot-obstacle collisions for failure detection |

### Without These Plugins

Collision detection won't work  
Physics won't update properly  
Robot will pass through obstacles without triggering failures  
ROS bridge topics won't publish correctly

### Location of World Files

BARN world files should be placed in:
```
~/ros_ws/src/turtlebot4_ignition_bringup/worlds/
```

**Important:** This modification is **mandatory** for the BARN dataset. The original files from the BARN repository are missing these plugins and will not work correctly with the TurtleBot4 simulation without this fix.

## Configuration Reference (`config.yaml`)

All system parameters are centralized in `~/ros_ws/config.yaml`. This file controls data generation, training, and evaluation settings.

---

### Data Generation Parameters

#### Obstacle Configuration
```yaml
NUM_VALID_OBSTACLES: 20    # Number of obstacles considered "active" for LiDAR simulation
OFFSET: 1.0                # Corridor half-width (meters) - distance from path centerline to obstacles
RADIUS: 0.4                # Obstacle radius (meters) - size of hallucinated circular obstacles
```

**What these mean:**
- **`NUM_VALID_OBSTACLES`**: Maximum number of nearest obstacles used in ray-tracing simulation (larger = more accurate but slower)
- **`OFFSET`**: Controls corridor width. The robot navigates down the center with obstacles placed at ±`OFFSET` meters perpendicular to the path
- **`RADIUS`**: Size of each obstacle. Smaller values = tighter corridors, harder navigation

**Example visualization:**
```
        ● RADIUS=0.4m
        │
        │ OFFSET=1.0m
        ├──────────────┤
    ●───────────────────────● 
        │   Robot Path  │
    ●───────────────────────●
        └── Corridor ───┘
```

**Recommended values:**
- **Tight corridors:** `OFFSET=0.6`, `RADIUS=0.3`
- **Standard (default):** `OFFSET=1.0`, `RADIUS=0.4`
- **Wide corridors:** `OFFSET=1.5`, `RADIUS=0.5`

---

### Random Walk Data Collection
```yaml
RANDOM_WALK_BAG_DKR: readme_example    # Output directory for raw bag files
```

**Purpose:** Specifies where random walk ROS bags are stored during data collection.

**Full path:** `~/ros_ws/ros_bag/readme_example/`

**Used by:** `training.launch.py` when recording expert demonstrations

---

### Dataset Generation
```yaml
TRAINING_DKR: ros_bag/readme_example   # Input directory containing raw bags to process
ASYM_FLAG: 1                           # 0 = symmetric obstacles, 1 = asymmetric (curvature-aware)
```

**Parameters:**
- **`TRAINING_DKR`**: Path to raw bag files (typically matches `RANDOM_WALK_BAG_DKR`)
- **`ASYM_FLAG`**: Controls obstacle placement strategy
  - `0` = **Symmetric**: Obstacles placed symmetrically on both sides of path
  - `1` = **Asymmetric**: Obstacles adjust based on path curvature (inside wall pulls in, outside wall widens)

**Asymmetric vs Symmetric:**
```
Symmetric (ASYM_FLAG=0):          Asymmetric (ASYM_FLAG=1):
    ●     ●     ●                      ●   ●     ●
    │     │     │                      │    │      │
────┼─────┼─────┼────            ─────┼────┼──────┼────
    │     │     │                           │    │  │
    ●     ●     ●                           ●  ●   ●
  (equal spacing)                    (wider on outside of turn)
```

**When to use each:**
- **Symmetric:** Simpler, more predictable environments
- **Asymmetric:** More realistic, models real-world corridor dynamics

---

### Model Evaluation
```yaml
CSV_FILES:
  - best_model_2026_validation/baseline.csv   # Where to save test results

MODEL_PATHS:
  - "created_models/gauss_2_asym_combo"       # Path to trained model(s) for testing

WORLD_NUMS: [12, 36, 75, 125, 203, 210, 69, 187, 266]   # BARN world IDs to test

NUM_TRIALS: 5           # Number of trials per world
RESET_DELAY: 3          # Seconds to wait between trials for system to stabilize
```

#### CSV Files

**Active entries** (uncommented):
```yaml
CSV_FILES:
  - best_model_2026_validation/baseline.csv
```

**Commented entries** (examples):
```yaml
# - readme_example/baseline.csv
# - poc_readme/baseline.csv
```

**Purpose:** Defines where experimental results are saved. Each entry creates a `baseline.csv` file in the specified directory under `~/ros_ws/results/`.

**Output location:** `~/ros_ws/results/best_model_2026_validation/baseline.csv`

**CSV columns:**
```
world_num, trial_num, trial_result, local_goal_reached, num_lg, TRIAL_TIME, 
CMD_AVG_LIN, CMD_AVG_ANG, ODOM_AVG_LIN, ODOM_AVG_ANG
```

---

#### Model Paths

**Active model:**
```yaml
MODEL_PATHS:
  - "created_models/gauss_2_asym_combo"
```

**Commented examples:**
```yaml
# - created_models/gauss_2_200_sym
# - created_models/readme_example_large
# - created_models/cnn_sym
# - created_models/cnn
```

**Purpose:** List of models to evaluate. Multiple models can be tested sequentially by uncommenting entries.

**Path resolution:**
- Relative paths resolve to: `~/ros_ws/created_models/`
- Absolute paths work as-is

**Expected structure:**
```
~/ros_ws/created_models/gauss_2_asym_combo/
├── model.onnx          # ONNX runtime model (required)
├── model.h5            # Keras checkpoint (optional)
└── config.yaml         # Model metadata (optional)
```

---

#### Test Worlds
```yaml
WORLD_NUMS: [12, 36, 75, 125, 203, 210, 69, 187, 266]
```

**Purpose:** BARN dataset world IDs to test against. Ordered by difficulty (easy → medium → hard).

**Difficulty breakdown:**
- **Easy:** `12, 36, 75` - Wide corridors, gentle curves
- **Medium:** `125, 203, 210` - Mixed complexity
- **Hard:** `69, 187, 266` - Tight spaces, sharp turns

**Commented alternative:**
```yaml
# WORLD_NUMS: [12, 36, 75]   # Quick test on easy worlds only
```

**Custom world sets:**
```yaml
# Full BARN benchmark (300 worlds):
WORLD_NUMS: [0, 1, 2, ..., 299]

# Specific challenging subset:
WORLD_NUMS: [69, 187, 266, 42, 156]
```

---

#### Trial Parameters
```yaml
NUM_TRIALS: 5          # Repetitions per world for statistical significance
RESET_DELAY: 3         # Cooldown between trials (seconds)
```

**NUM_TRIALS:**
- Higher values = better statistics, longer runtime
- **Recommended:** 5-10 for validation, 3 for quick tests

**RESET_DELAY:**
- Time for Gazebo physics to stabilize between trials
- **Too short:** Robot may not reset properly
- **Too long:** Wastes time
- **Recommended:** 3-5 seconds

**Runtime estimation:**
```
Total time ≈ (NUM_WORLDS × NUM_TRIALS × AVG_TRIAL_TIME) + (NUM_WORLDS × NUM_TRIALS × RESET_DELAY)

Example:
  9 worlds × 5 trials × 60s avg + (9 × 5 × 3s) = 2,835s ≈ 47 minutes
```

---

### Quick Configuration Examples

#### Example 1: Generate Training Data (Asymmetric)
```yaml
RANDOM_WALK_BAG_DKR: my_dataset
TRAINING_DKR: ros_bag/my_dataset
ASYM_FLAG: 1
OFFSET: 1.0
RADIUS: 0.4
```

#### Example 2: Quick Model Test (3 Easy Worlds)
```yaml
MODEL_PATHS:
  - created_models/my_test_model
WORLD_NUMS: [12, 36, 75]
NUM_TRIALS: 3
CSV_FILES:
  - quick_test/baseline.csv
```

#### Example 3: Full Benchmark (9 Worlds, High Confidence)
```yaml
MODEL_PATHS:
  - created_models/final_model
WORLD_NUMS: [12, 36, 75, 125, 203, 210, 69, 187, 266]
NUM_TRIALS: 10
RESET_DELAY: 5
CSV_FILES:
  - final_benchmark/baseline.csv
```

---

### Configuration Workflow

1. **Data Collection Phase:**
```yaml
   RANDOM_WALK_BAG_DKR: my_experiment
```

2. **Dataset Generation Phase:**
```yaml
   TRAINING_DKR: ros_bag/my_experiment
   ASYM_FLAG: 1  # Choose obstacle style
```

3. **Training Phase:**
   - Use `neural_net.py` with data from `TRAINING_DKR`
   - Save model to `created_models/my_model`

4. **Evaluation Phase:**
```yaml
   MODEL_PATHS:
     - created_models/my_model
   WORLD_NUMS: [12, 36, 75, ...]
   NUM_TRIALS: 5
   CSV_FILES:
     - my_experiment_results/baseline.csv
```

5. **Analysis Phase:**
```bash
   python3 trial_summary.py results/my_experiment_results
```

## Created Models Directory

This directory contains all trained neural network controllers evaluated in this thesis. Each model represents a different training configuration, architecture, or dataset variation tested during development.

**Location:** `~/ros_ws/created_models/`

### Model Inventory
```
created_models/
├── gauss_2_200_sym/         # MLP trained on 200k symmetric obstacle dataset
├── gauss_2_asym_combo/      # MLP trained on combined asymmetric dataset
├── cnn/                     # CNN trained on asymmetric obstacles
├── cnn_sym/                 # CNN trained on symmetric obstacles
├── readme_example_large/    # Large-scale MLP model (demonstration dataset)
└── readme_example_model/    # Small MLP model (tutorial/quickstart)
```

### Model Naming Convention

Models follow the pattern: `{architecture}_{dataset}_{obstacle_type}`

- **Architecture:** `mlp` (Multi-Layer Perceptron) or `cnn` (Convolutional Neural Network)
- **Dataset:** `gauss_2_200` (200k samples), `readme_example` (demo data)
- **Obstacle Type:** `sym` (symmetric), `asym` (asymmetric/curvature-aware)

## Training Data Overlay

A utility script for visualizing and comparing robot trajectories from multiple ROS2 bag files. Useful for verifying random walk quality, analyzing navigation patterns, and debugging data collection issues.

**Location:** `~/ros_ws/training_paths_overlay.py`

### Purpose

Overlays `/odom` XY paths from one or more rosbag2 directories onto a single plot, allowing visual comparison of:
- Random walk exploration coverage
- Path diversity across multiple runs
- Data collection quality (smooth vs. erratic movements)
- Dataset characteristics before training

### Usage
```bash
python3 training_path_overlay.py
```

### Basic Examples

**Single bag:**
```bash
python3 training_path_overlay.py ros_bag/readme_example/2026-01-01_12-40-05_gaus
```

**Multiple bags (entire directory):**
```bash
python3 training_path_overlay.py ros_bag/readme_example
```

**Normalized origins (all paths start at 0,0):**
```bash
python3 training_path_overlay.py ros_bag/readme_example --normalize
```

**Save to file instead of displaying:**
```bash
python3 training_path_overlay.py ros_bag/readme_example --out path_overlay.png
```

**Downsample for large bags (faster, less memory):**
```bash
python3 training_path_overlay.py ros_bag/readme_example --downsample 10
# Keeps 1 of every 10 odometry messages
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `<root>` | *required* | Path to a single bag or directory containing multiple bags |
| `--odom` | `/odom` | Topic name to read odometry from |
| `--downsample N` | `1` | Sample rate reduction (1 = all points, 10 = every 10th point) |
| `--normalize` | `False` | Shift all paths so they start at origin (0,0) |
| `--out <file>` | *(show plot)* | Save to image file instead of displaying interactively |

### Output Interpretation

**Plot elements:**
- **Colored lines:** Individual robot paths (one color per bag file)
- **Circle (○):** Start position of each path
- **Cross (×):** End position of each path
- **Legend:** Bag file timestamps (if not too many)

## Automated Testing Script (`multiple_scripts.sh`)

**Location:** `~/ros_ws/multiple_scripts.sh`

### Purpose

Automated batch testing framework that evaluates trained models across multiple BARN worlds and trials. This script orchestrates the complete testing pipeline: model deployment, world loading, trial execution, and result collection.

### What It Does

**High-level workflow:**
1. Reads configuration from `config.yaml` (models, worlds, trial counts)
2. For each model:
   - Deploys ONNX model and scaler files to runtime directory
   - Determines architecture (MLP vs CNN) based on file presence
   - Rebuilds controller with correct variant
3. For each world:
   - Copies world `.sdf` file and map files to appropriate locations
   - Runs N trials (configured via `NUM_TRIALS`)
   - Records results to model-specific CSV file
4. Handles failures gracefully:
   - Retries trials on AMCL timeouts
   - Skips counting failed navigation server starts
   - Cleans up Gazebo processes between trials

### Usage

**Basic execution:**
```bash
./multiple_scripts.sh
```

**Prerequisites:**
- Models in `~/ros_ws/created_models/`
- BARN world files in `~/ros_ws/BARN_turtlebot/world_files/`
- BARN map files in `~/ros_ws/BARN_turtlebot/map_files/`
- `config.yaml` properly configured

### Configuration (via `config.yaml`)

The script reads these parameters:
```yaml
MODEL_PATHS:              # List of models to test
  - created_models/gauss_2_asym_combo
  - created_models/cnn_sym

CSV_FILES:                # Output CSV per model (auto-generated if omitted)
  - results/model1/baseline.csv
  - results/model2/baseline.csv

WORLD_NUMS:               # BARN worlds to test
  [12, 36, 75, 125, 203, 210, 69, 187, 266]

NUM_TRIALS: 5             # Trials per world
RESET_DELAY: 3            # Cooldown between trials (seconds)
```

### Model Detection Logic

**Automatic architecture detection:**
- If scaler files (`*_scaler_mins.txt`, `*_scaler_maxs.txt`) exist → **MLP mode**
- If scaler files missing → **CNN mode**
- Rebuilds ONNX controller package with correct `CONTROLLER_VARIANT` flag

### Output

**Console output:**
```
=== MODEL[0]: created_models/gauss_2_asym_combo -> CSV: results/model1/baseline.csv ===
Staged model: gauss_2_asym_combo_mlp_model.onnx
  scaler_mins.txt
  scaler_maxs.txt

=== Starting experiments for world 12 (model idx 0) ===
___ Starting Trial 1 for world 12 ___
  ✓ Trial 1 completed with result: SUCCESS

___ Starting Trial 2 for world 12 ___
  ✗ Trial 2 completed with result: COLLISION
...
```

**CSV output** (one per model):
```csv
world_num,trial_num,trial_result,local_goal_reached,num_lg,TRIAL_TIME,CMD_AVG_LIN,CMD_AVG_ANG,ODOM_AVG_LIN,ODOM_AVG_ANG
12,1,SUCCESS,45,45,34.2,0.18,0.12,0.17,0.11
12,2,COLLISION,23,45,18.5,0.21,0.15,0.20,0.14
...
```

### Key Features

**Robustness:**
-  Automatic Gazebo cleanup between trials
-  Retry logic for AMCL/navigation failures
-  Per-model CSV isolation (prevents data mixing)
-  Graceful handling of missing scaler files

**Flexibility:**
-  Test multiple models sequentially
-  Mix MLP and CNN architectures
-  Customize trial counts and delays
-  Resume from failures (CSV appends results)

**Deployment:**
-  Versioned model staging (preserves history)
-  Symlink-based "current model" switching
-  Automatic controller recompilation per architecture

### Trial Result Handling

The script monitors each trial's outcome and takes appropriate action:

| Result | Action |
|--------|--------|
| `SUCCESS` | Count trial, continue |
| `COLLISION` | Count trial, continue |
| `TIMEOUT` | Count trial, continue |
| `AMCL TIMEOUT - MAX RETRIES EXCEEDED` | Retry (don't count) |
| `NAV_SERVER_UNAVAILABLE` | Retry (don't count) |

### Runtime Estimation
```
Total time ≈ NUM_MODELS × NUM_WORLDS × NUM_TRIALS × (AVG_TRIAL + RESET_DELAY)

Example:
  2 models × 9 worlds × 5 trials × (60s + 3s) = 5,670s ≈ 1.6 hours
```

### Troubleshooting

**Script exits immediately:**
- Check model paths in `config.yaml` are correct
- Verify `.onnx` files exist in model directories

**Gazebo not cleaning up:**
- Increase `RESET_DELAY` to 5+ seconds
- Manually kill: `pkill -9 -f "ign gazebo"`

**Wrong controller variant:**
- Verify scaler files are named: `*_scaler_mins.txt` and `*_scaler_maxs.txt`
- Check console output for "MLP" or "CNN" mode confirmation

**CSV not created:**
- Ensure output directory exists: `mkdir -p ~/ros_ws/results/model_name`
- Check file permissions in `~/ros_ws/`

### Integration with Analysis

After completion, analyze results with:
```bash
python3 trial_summary.py results/model1
python3 trial_summary.py results/model2
```

This generates comparison plots and statistics for each tested model.

---

**Note:** This script is designed for hands-off overnight testing. It can run for hours/days depending on configuration. Monitor initial trials to ensure proper setup before leaving unattended.
