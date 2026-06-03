cmake_minimum_required(VERSION 3.8)
project(ship_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 定義你的服務檔案
set(srv_files
  "srv/Fire.srv"
)

# 產生介面
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
)

# 匯出執行依賴 (這讓其他套件能找到此套件)
ament_export_dependencies(rosidl_default_runtime)

ament_package()