cmake_minimum_required(VERSION 3.5)
project(drone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# メッセージファイルの設定
set(msg_files
  "msg/DroneState.msg"
  "msg/DroneControlCommand.msg"
)

# ROS2標準のメッセージ生成設定
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# 生成されたメッセージ用のランタイム依存関係をエクスポート
ament_export_dependencies(rosidl_default_runtime)

ament_package()