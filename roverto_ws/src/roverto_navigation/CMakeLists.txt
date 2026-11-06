cmake_minimum_required(VERSION 3.8)
project(roverto_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install config and launch directories to the package share folder
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
