cmake_minimum_required(VERSION 3.5)
project(pose_saver)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS
  pose_saver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
)
