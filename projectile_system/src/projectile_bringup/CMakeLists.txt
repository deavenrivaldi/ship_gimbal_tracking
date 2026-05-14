cmake_minimum_required(VERSION 3.8)
project(projectile_bringup)

find_package(ament_cmake REQUIRED)


# Install launch files

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Install config files (YAML)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()