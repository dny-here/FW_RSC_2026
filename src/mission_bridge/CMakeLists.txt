cmake_minimum_required(VERSION 3.8)
project(mission_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(mission_bridge_node src/mission_bridge_node.cpp)
ament_target_dependencies(mission_bridge_node
  rclcpp std_msgs sensor_msgs geographic_msgs mavros_msgs interfaces)

install(TARGETS mission_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_fsm test/test_mission_fsm.cpp)
  target_include_directories(test_mission_fsm PRIVATE include)
endif()

ament_package()
