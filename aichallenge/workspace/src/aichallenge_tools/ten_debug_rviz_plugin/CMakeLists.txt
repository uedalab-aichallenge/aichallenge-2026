cmake_minimum_required(VERSION 3.14)
project(ten_debug_rviz_plugin)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/layers_panel.cpp
  include/ten_debug_rviz_plugin/layers_panel.hpp
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(INSTALL_TO_SHARE plugins)
