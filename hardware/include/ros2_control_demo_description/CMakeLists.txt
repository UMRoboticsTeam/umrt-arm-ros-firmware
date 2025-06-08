cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY prairie_pioneer/urdf prairie_pioneer/rviz
  DESTINATION share/${PROJECT_NAME}/prairie_pioneer
)

ament_package()
