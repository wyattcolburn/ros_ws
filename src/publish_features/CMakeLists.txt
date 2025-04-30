cmake_minimum_required(VERSION 3.10...3.18)
project(publish_features)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)

# Create an executable from the source file
add_executable(${PROJECT_NAME}_node src/approximate.cpp)

# Link against ROS2 packages
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
  tf2
)

# Link against raylib
target_link_libraries(${PROJECT_NAME}_node)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

