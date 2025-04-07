cmake_minimum_required(VERSION 3.10)
project(onnx)

# Set ONNX Runtime paths once
set(ONNXRUNTIME_PATH $ENV{HOME}/onnxruntime/onnxruntime-linux-x64-1.16.3)
include_directories(${ONNXRUNTIME_PATH}/include)
link_directories(${ONNXRUNTIME_PATH}/lib)

# Enable C++17 for ONNX Runtime
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler Settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)  # Added for Float64MultiArray

nav2_package()

include_directories(
  include
)

# List of dependencies
set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  std_msgs  # Added for Float64MultiArray
)

# Create ONNX Plugin
add_library(onnx_controller SHARED src/onnx_controller.cpp src/mpc.cpp)

# Prevent plugin from using boost
target_compile_definitions(onnx_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Link ONNX Runtime
target_link_libraries(onnx_controller onnxruntime)

# Link ROS2 Dependencies
ament_target_dependencies(onnx_controller 
  ${dependencies}
)

# Install the compiled library
install(TARGETS onnx_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install ONNX headers - updated to use correct path
install(DIRECTORY ${ONNXRUNTIME_PATH}/include/
  DESTINATION include/onnxruntime
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export plugin info
ament_export_include_directories(include)
ament_export_libraries(onnx_controller)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core onnx_controller.xml)
ament_package()
