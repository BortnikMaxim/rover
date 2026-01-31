cmake_minimum_required(VERSION 3.8)
project(delivery_robot_sim)

find_package(ament_cmake REQUIRED)

# Install resources so ros2 can find them from install/ share/
install(DIRECTORY
  urdf
  launch
  rviz
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
