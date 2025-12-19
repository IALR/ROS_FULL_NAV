cmake_minimum_required(VERSION 3.8)
project(robotics_sim_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories so ROS 2 can find them
install(DIRECTORY
  worlds
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
