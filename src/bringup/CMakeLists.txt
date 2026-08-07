cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament build tools
find_package(ament_cmake REQUIRED)

# Install the launch and config directories into the package's share folder.
# config/ berisi bridge.yaml: dulu file ini ada di root workspace dan dimuat
# lewat path relatif, sehingga launch hanya jalan kalau kebetulan dijalankan
# dari root. Sekarang dipasang ke share/ dan dirujuk dengan path absolut.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()