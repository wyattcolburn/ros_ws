#!/bin/bash

# creating a map
ros2 launch turtlebot4_navigation slam.launch.py


#bring up rviz2 to visualize the map being created
ros2 launch turtlebot4_viz view_robot.launch.py


#save the map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"


