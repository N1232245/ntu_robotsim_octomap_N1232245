cmake_minimum_required(VERSION 3.5)
project(ntu_robotsim_octomap)

find_package(ament_cmake REQUIRED)

# Install resources used by Gazebo / launch / configs
install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_resource_path.dsv")

ament_package()
