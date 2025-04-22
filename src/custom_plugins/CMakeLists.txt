cmake_minimum_required(VERSION 3.8)
project(custom_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(CustomDiffDrivePlugin
  SHARED
  src/diff_drive_plugin.cpp)

# Gazebo packages are CMake packages which export their include directories when we link against them
target_link_libraries(CustomDiffDrivePlugin
  gz-sim8::core
  gz-plugin2::gz-plugin2)

ament_target_dependencies(CustomDiffDrivePlugin
  rclcpp
  geometry_msgs
)

ament_export_libraries(CustomDiffDrivePlugin)

install(TARGETS CustomDiffDrivePlugin 
  LIBRARY
  DESTINATION lib
)

ament_package()