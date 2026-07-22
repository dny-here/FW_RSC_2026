cmake_minimum_required(VERSION 3.8)
project(mission_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_fsm test/test_mission_fsm.cpp)
  target_include_directories(test_mission_fsm PRIVATE include)
endif()

ament_package()
