cmake_minimum_required(VERSION 3.8)
project(airdrop_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_executable(airdrop_planner_node src/airdrop_planner_node.cpp)
ament_target_dependencies(airdrop_planner_node
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs interfaces)

install(TARGETS airdrop_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ballistic_model test/test_ballistic_model.cpp)
  target_include_directories(test_ballistic_model PRIVATE include)
  ament_add_gtest(test_geo_utils test/test_geo_utils.cpp)
  target_include_directories(test_geo_utils PRIVATE include)
endif()

ament_package()
