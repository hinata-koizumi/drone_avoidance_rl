cmake_minimum_required(VERSION 3.5)
project(state_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_msgs REQUIRED)

# Pythonモジュールのインストール
ament_python_install_package(${PROJECT_NAME})

# 実行ファイルのインストール
# install(TARGETS
#   state_bridge_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# 起動ファイルのインストール
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package() 