# LFH Thesis: Neural Network Navigation Controller

> Replacing DWA local controller with a neural network trained on hallucinated LiDAR data for autonomous robot navigation

## 🛠️ Tech Stack

- **ROS 2 Humble** - Robot Operating System
- **C++** - Controller implementation
- **Python** - Data processing and training
- **Keras/TensorFlow** - Neural network training
- **ONNX** - Model deployment
- **Gazebo Ignition** - Simulation environment

---

## 📋 Table of Contents

- [Setup Modifications](#setup-modifications)
- [Creating Maps](#creating-maps)
- [Generating Training Data](#generating-training-data)
- [Converting ROS Bags](#converting-ros-bags-to-training-data)
- [Dataset Information](#dataset-information)

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

## 🎲 Generating Training Data

### Random Walk Expert Policy

Random walks provide the expert behavior that the learning policy will imitate.
```bash
ros2 launch my_robot_bringup training.launch.py
```

### Configuration

**Robot spawn timing:**
```python
period=15.0,  # 15 second delay before spawning random walk
actions=[Node(
    package='my_robot_bringup',
    executable='gaussian',
    output='screen')]
```

**Bag storage location:**  
Defined in `config.yaml` under `RANDOM_WALK_BAG_DKR`

**Recorded topics:**
```python
bag_record = ExecuteProcess(
    cmd=[
        'ros2', 'bag', 'record',
        '/scan',        # LiDAR data
        '/tf',          # Transform tree
        '/tf_static',   # Static transforms
        '/odom',        # Odometry
        '/cmd_vel',     # Velocity commands
        '/clock',       # Simulation time
        '-o', bag_output
    ],
    output='screen'
)
```

---

## 📊 Converting ROS Bags to Training Data

### Processing Script

Location: `~/ros_ws/src/my_robot_bringup/my_robot_bringup/multiple_seg.py`

### Prerequisites

Start the following nodes in separate terminals:
```bash
# Terminal 1: Start simulator
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py

# Terminal 2: Start localization
ros2 launch turtlebot4_navigation localization.launch.py \
    map:=big_map_april_4.yaml \
    use_sim_time:=true

# Terminal 3: Start Nav2 (must use DWA, not custom controller)
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true

# Terminal 4: Run processing script
ros2 run my_robot_bringup multiple
```
---


**Worlds for TurleBot:** All worlds need exist within turtlebot4_ignition_bringup/worlds directory. There is a weird bug where only the present names
work for live topics

Metric values
The metrics_files folder contains the 5 difficulty metrics calculated on the path in this order: distance to closest obstacle, average visibility, dispersion, characteristic dimension, and tortuosity

## TurtleBot4 Setup Modifications

### No Dock Spawning
Modified `turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py` to remove charging dock spawn (not needed for navigation testing).

See diff against original:
```
diff src/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py \
     /opt/ros/humble/share/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py


```

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

---

### Important Notes

⚠️ **After editing `config.yaml`:**
- No rebuild required (read at runtime)
- Changes take effect immediately on next launch

⚠️ **Path conventions:**
- All relative paths resolve from `~/ros_ws/`
- Use forward slashes `/` even on Windows (if applicable)

⚠️ **Comments in YAML:**
- Use `#` to disable entries
- Keep commented entries as examples for future use
