cmake_minimum_required(VERSION 3.16)
project(model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Remove the find_package(onnx_inference REQUIRED) line

# Find ONNX Runtime
find_path(ONNXRUNTIME_INCLUDE_DIRS
  NAMES onnxruntime_cxx_api.h
  PATHS /usr/local/include/onnxruntime
)
find_library(ONNXRUNTIME_LIBRARIES
  NAMES onnxruntime
  PATHS /usr/local/lib /usr/lib
)

if(NOT ONNXRUNTIME_INCLUDE_DIRS OR NOT ONNXRUNTIME_LIBRARIES)
  message(FATAL_ERROR "ONNX Runtime not found. ")
endif()
if(NOT ONNXRUNTIME_LIBRARIES)
  message(FATAL_ERROR "ONNX Runtime library not found. Check installation.")
endif()

message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARIES}")
# Include directories
include_directories(
  include
  ${ONNXRUNTIME_INCLUDE_DIRS}
)

# Add the ONNX model library
add_library(onnx_model src/onnx_model.cpp)
target_include_directories(onnx_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(onnx_model ${ONNXRUNTIME_LIBRARIES})

# Add the ROS node
add_executable(inference_node src/main.cpp)
target_link_libraries(inference_node onnx_model)
ament_target_dependencies(inference_node 
  rclcpp sensor_msgs geometry_msgs std_msgs nav_msgs ament_index_cpp)

# Install targets
install(TARGETS
  onnx_model
  inference_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
