cmake_minimum_required(VERSION 3.5)
project(aichallenge_submit_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS
  scripts/ten_tune_tui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch map data config)
