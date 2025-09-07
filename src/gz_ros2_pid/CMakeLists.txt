cmake_minimum_required(VERSION 3.8)
project(gz_ros2_pid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_toolbox REQUIRED)

# GZ_ROS2_CONTROL BASE PLUGIN LIBRARY

add_library(
  gz_ros2_control_plugin
  SHARED
  src/gz_ros2_control/gz_ros2_control_plugin.cpp
  src/gz_ros2_control/gz_system.cpp
)

# Gazebo packages are CMake packages which export their include
# directories when we link against them
target_link_libraries(
  gz_ros2_control_plugin
  gz-sim8::core
  gz-plugin2::gz-plugin2
)

# Use generator expressions to specify package include path at generate time for build and install contexts
target_include_directories(
  gz_ros2_control_plugin
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # Relative to source path
  $<INSTALL_INTERFACE:include>  # Relative to installation prefix
)

# Add include directories and link against target for ROS 2 packages
ament_target_dependencies(
  gz_ros2_control_plugin
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
)

ament_export_libraries(gz_ros2_control_plugin)

install(
  TARGETS gz_ros2_control_plugin
  LIBRARY
  DESTINATION lib
)

# GZ_ROS2_PID DERIVED PLUGIN LIBRARY

add_library(
  gz_ros2_pid_plugin
  SHARED
  src/gz_ros2_pid/gz_ros2_pid_plugin.cpp
  src/gz_ros2_pid/gz_pid_system.cpp
)

target_link_libraries(
  gz_ros2_pid_plugin
  gz_ros2_control_plugin
  gz-sim8::core
  gz-plugin2::gz-plugin2
)

target_include_directories(
  gz_ros2_pid_plugin
  PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
  $<INSTALL_INTERFACE:include>  
)

ament_target_dependencies(
  gz_ros2_pid_plugin
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  control_toolbox
)

ament_export_libraries(gz_ros2_pid_plugin)

install(
  TARGETS gz_ros2_pid_plugin
  LIBRARY
  DESTINATION lib
)

# Make gz_ros2_pid::GazeboPidSimSystem plugin discoverable by pluginlib
pluginlib_export_plugin_description_file(
  gz_ros2_control  # Export under same namespace as GazeboSimSystemInterface to enable discovery by gz_ros2_control::GZResourceManager
  plugin_description.xml
)
install(
  FILES plugin_description.xml
  DESTINATION share/gz_ros2_pid
)

ament_package()
