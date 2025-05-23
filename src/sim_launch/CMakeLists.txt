cmake_minimum_required(VERSION 3.8)
project(sim_launch)

find_package(ament_cmake REQUIRED)

# package.xmlも必ずインストール
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "package.xml"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 