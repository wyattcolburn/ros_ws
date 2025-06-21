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
**Worlds for TurleBot:** All worlds need exist within turtlebot4_ignition_bringup/worlds directory. There is a weird bug where only the present names
work for live topics

