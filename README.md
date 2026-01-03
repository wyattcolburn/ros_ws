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
