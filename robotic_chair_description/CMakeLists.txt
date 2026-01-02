cmake_minimum_required(VERSION 3.8)
project(robotic_chair_description)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install config files (if you have any)
# install(
#   DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# Install worlds (if you have any)
# install(
#   DIRECTORY worlds/
#   DESTINATION share/${PROJECT_NAME}/worlds
#   FILES_MATCHING PATTERN "*.world"
# )

# Install meshes (if you have any)
# install(
#   DIRECTORY meshes/
#   DESTINATION share/${PROJECT_NAME}/meshes
# )

ament_package()

