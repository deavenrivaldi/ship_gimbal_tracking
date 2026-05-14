cmake_minimum_required(VERSION 3.8)
project(projectile_msgs)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# Génération des interfaces (srv/msg)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Fire.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Export des dépendances runtime
ament_export_dependencies(rosidl_default_runtime)

ament_package()
