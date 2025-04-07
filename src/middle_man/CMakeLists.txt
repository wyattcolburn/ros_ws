cmake_minimum_required(VERSION 3.10...3.18)
project(middle_man)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Create an executable from the source file
add_executable(${PROJECT_NAME}_node src/main.cpp src/mpc.cpp src/obstacles.cpp src/raytracing.cpp)

# Link against ROS2 packages
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Link against raylib
target_link_libraries(${PROJECT_NAME}_node raylib)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

