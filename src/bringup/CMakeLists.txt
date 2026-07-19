cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament build tools
find_package(ament_cmake REQUIRED)

# Install the launch directory into the package's share folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()