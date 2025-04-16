cmake_minimum_required(VERSION 3.5)
project(map_pkg)

# Find dependencies
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package()

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

