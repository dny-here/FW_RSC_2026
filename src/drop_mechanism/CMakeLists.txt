cmake_minimum_required(VERSION 3.8)
project(drop_mechanism)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(drop_relay_node src/drop_relay_node.cpp)
ament_target_dependencies(drop_relay_node
  rclcpp mavros_msgs interfaces)

install(TARGETS drop_relay_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
