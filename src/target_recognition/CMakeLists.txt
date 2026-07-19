cmake_minimum_required(VERSION 3.8)
project(target_recognition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Custom message/action interfaces
find_package(interfaces REQUIRED)

# ROS 2 Actions
find_package(rclcpp_action REQUIRED) 


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Build Target
add_executable(target_detection src/targetdetection.cpp)

# Include Directories & Linking
target_include_directories(target_detection PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

# Dependency List
ament_target_dependencies(target_detection 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  cv_bridge 
  image_transport 
  tf2 
  tf2_ros 
  interfaces
  rclcpp_action 
)

# Link OpenCV
target_link_libraries(
  target_detection ${OpenCV_LIBS}
  rclcpp_action::rclcpp_action
)

# Install Directives
# Install the executable
install(TARGETS target_detection
  DESTINATION lib/${PROJECT_NAME})

# Install the header files so the workspace can find them
install(DIRECTORY include/
  DESTINATION include
)

ament_package()