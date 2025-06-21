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

