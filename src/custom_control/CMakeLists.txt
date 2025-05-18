cmake_minimum_required(VERSION 3.8)
project(custom_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  twist_to_twist_stamped
  src/twist_to_twist_stamped.cpp 
)

ament_target_dependencies(
  twist_to_twist_stamped
  rclcpp
  std_msgs
  geometry_msgs
)

install(
  TARGETS twist_to_twist_stamped
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
