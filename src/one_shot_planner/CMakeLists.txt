
cmake_minimum_required(VERSION 3.8)
project(one_shot_planner)

# ==============================
# Compiler settings
# ==============================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# ==============================
# Dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

# Collect ament deps (used for export + target deps)
set(AMENT_DEPS
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  builtin_interfaces
  tf2_ros
)

# ==============================
# Library (the plugin)
# ==============================
# Name the library distinctly; this must match the <library path="..."> in your XML
set(library_name ${PROJECT_NAME}_plugin)

add_library(${library_name} SHARED
  src/one_shot_planner.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name} ${AMENT_DEPS})

# Optional: silences Boost-related pluginlib warnings on some distros
target_compile_definitions(${library_name} PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# ==============================
# Pluginlib export (XML)
# ==============================
# Make sure one_shot_planner.xml exists at the package root and that its
# <library path="..."> equals "${library_name}" (i.e., "one_shot_planner_plugin")
pluginlib_export_plugin_description_file(nav2_core one_shot_planner.xml)

# ==============================
# Install
# ==============================
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES one_shot_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

# ==============================
# Export for downstream packages
# ==============================
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${AMENT_DEPS})

# ==============================
# Tests (optional)
# ==============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
