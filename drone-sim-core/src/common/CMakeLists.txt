cmake_minimum_required(VERSION 3.5)
project(common)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Pythonモジュールのインストール
ament_python_install_package(${PROJECT_NAME})

ament_package() 