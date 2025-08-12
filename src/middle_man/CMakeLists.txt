cmake_minimum_required(VERSION 3.10...3.18)
project(middle_man)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# Create an executable from the source file
add_executable(${PROJECT_NAME}_node src/main.cpp src/mpc.cpp src/obstacles.cpp src/raytracing.cpp)
add_executable(${PROJECT_NAME}_valid src/obs_validation.cpp src/obstacles.cpp src/raytracing.cpp)
add_executable(${PROJECT_NAME}_barn src/barn_time.cpp src/obstacles.cpp src/raytracing.cpp)


# Link against ROS2 packages
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME}_valid
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
)
ament_target_dependencies(${PROJECT_NAME}_barn
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_barn yaml-cpp)

install(TARGETS 
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_valid
  ${PROJECT_NAME}_barn
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

