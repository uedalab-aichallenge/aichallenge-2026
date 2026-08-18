cmake_minimum_required(VERSION 3.8)
project(act_lerobot_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  act_lerobot_controller/act_lerobot_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  ckpt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
