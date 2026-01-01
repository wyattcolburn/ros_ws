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
    executable='gaussian_random_walk',
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
## Important Commands:

**Creating a map:** 
1) start the world
2) start slam
3) open rviz
4) move robot around with keyboard
5) save
```
ros2 launch my_robot_bringup simple_world.launch.py
ros2 launch turtlebot4_navigation slam.launch.py
ros2 launch turtlebot4_viz view_robot.launch.py
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"

```



**Converting a rosbag to training data**

The python file which does everything is ~/ros_ws/src/my_robot_bringup/my_robot_bringup/multiple_seg.py


However, you need to first need to 
1) start the simulator
2) start the localizator 
3) start nav2 (which should be using dwa not custom controller)
4) ros2 run my_robot_bringup multiple
```
# start the simulator
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py

# New window for localization
ros2 launch turtlebot4_navigation localization.launch.py map:=big_map_april_4.yaml use_sim_time:=true


# New window for nav2
ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=true



```
**Worlds for TurleBot:** All worlds need exist within turtlebot4_ignition_bringup/worlds directory. There is a weird bug where only the present names
work for live topics

Metric values
The metrics_files folder contains the 5 difficulty metrics calculated on the path in this order: distance to closest obstacle, average visibility, dispersion, characteristic dimension, and tortuosity



**Gauss 2 Explanations**

Gauss 2 refers to our second attempt at a gaussain based random walk policy 

gauss_2: part 1 of the dataset. around 75000 points that was trained on symmetrical obstacle placement with no added noise

gauss_2_noisy: refers to same 75,000 points but there was added noise to the symmetrical obstacle placement

guass_2_symetric: is the same 75,000 points but the obstacles are placed assymetrically based on the curvature of the path, inside wall and outside wall

guass_2_combo: I believe is guass_2, gauss_2_noisy, gauss_2_asymetric all combined into 1 dataset.
I am not sure what the difference between gauss_2_combo and gauss_2_combo_full is

guass_2_pt2: Is a new set of random walks, same rando walk policy as gauss_2 but just trying to add more data: no results

gauss_2_combined_raw: is currently trained with asymetric policy on the combination of gauss_2 and gauss_2 part 2. It is around 200000 data points, looks promising but only one 4 new worlds



gauss_2_combined_asy: Not sure what this is, I figure it is a the part1 and part2 combined trained in the same way that gauss_2_combined_raw is??

## TurtleBot4 Setup Modifications

### No Dock Spawning
Modified `turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py` to remove charging dock spawn (not needed for navigation testing).

See diff against original:
```bash
diff src/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py \
     /opt/ros/humble/share/turtlebot4_ignition_bringup/launch/turtlebot4_spawn.launch.py
``




