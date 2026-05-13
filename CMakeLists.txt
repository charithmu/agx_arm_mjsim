cmake_minimum_required(VERSION 3.8)
project(agx_arm_mjsim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf mjcf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
