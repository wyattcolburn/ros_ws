# LFH Thesis
Replacing DWA local controller with a neural network trained on hallucinated lidar



## How It's Made:

**Tech used:** ROS2, C++, Python, Keras, ONNX,


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

**Random walks:**
```
ros2 launch my_robot_bringup training.launch.py
```
This launch file starts the simulator but also calls the file undock_node: ~/ros_ws/src/my_robot_bringup/my_robot_bringup/undock_node.py

This starts a rosbag and does random walk

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
```


Removing all instances of dock (description, spawn, directory) always the robot to spawn with no dock. This elimates the need to undock before conduction random walks or experiments


