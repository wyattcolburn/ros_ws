cmake_minimum_required(VERSION 3.10)
project(onnx)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS 2 deps ---
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
find_package(std_msgs REQUIRED)

nav2_package()

set(dependencies
  rclcpp geometry_msgs nav2_costmap_2d pluginlib nav_msgs
  nav2_util nav2_core tf2 std_msgs
)

# --- Variant selection (compile-time) ---
set(CONTROLLER_VARIANT "$ENV{CONTROLLER_VARIANT}")
if(NOT CONTROLLER_VARIANT)
  set(CONTROLLER_VARIANT "MLP")
endif()
message(STATUS "Building controller variant: ${CONTROLLER_VARIANT}")

if(CONTROLLER_VARIANT STREQUAL "CNN")
  set(CONTROLLER_SRC src/cnn_controller.cpp)
else()
  set(CONTROLLER_SRC src/onnx_controller.cpp)
endif()

# --- ONNX Runtime (prefer discover over hardcoding) ---
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /usr/local/include /usr/include)
find_library(ONNXRUNTIME_LIB onnxruntime
  PATHS /usr/local/lib /usr/lib)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "Could not find ONNX Runtime (headers or lib).")
endif()

# --- Library (single target) ---
add_library(onnx_controller SHARED ${CONTROLLER_SRC} src/mpc.cpp)
target_compile_definitions(onnx_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_include_directories(onnx_controller
  PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(onnx_controller ${ONNXRUNTIME_LIB})
ament_target_dependencies(onnx_controller ${dependencies})

# --- Install & export ---
install(TARGETS onnx_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# One XML that always maps to the same library/class
pluginlib_export_plugin_description_file(nav2_core onnx_controller.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(onnx_controller)
ament_export_dependencies(${dependencies})
ament_package()
