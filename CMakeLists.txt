cmake_minimum_required(VERSION 3.8)
project(zlac_can_bringup)

find_package(ament_cmake REQUIRED)

# Only resources used by zlac_can_bringup.launch.py.
install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
