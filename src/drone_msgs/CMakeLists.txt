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
  ADD_LINTER_TESTS
)

# Pythonバージョン取得
execute_process(
  COMMAND python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
  OUTPUT_VARIABLE PYTHON_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REPLACE "." ";" PYTHON_VERSION_LIST ${PYTHON_VERSION})
list(GET PYTHON_VERSION_LIST 0 PYTHON_VERSION_MAJOR)
list(GET PYTHON_VERSION_LIST 1 PYTHON_VERSION_MINOR)

# Pythonバインディングのインストール
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/drone_msgs
  DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages
)

# 生成されたメッセージ用のランタイム依存関係をエクスポート
ament_export_dependencies(rosidl_default_runtime geometry_msgs builtin_interfaces)

ament_package()