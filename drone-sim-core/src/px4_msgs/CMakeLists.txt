cmake_minimum_required(VERSION 3.5)
project(px4_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 空のメッセージファイルリスト（実際のPX4メッセージは後で追加）
set(msg_files
)

# ROS2標準のメッセージ生成設定
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ADD_LINTER_TESTS
)

# 生成されたメッセージ用のランタイム依存関係をエクスポート
ament_export_dependencies(rosidl_default_runtime)

ament_package() 