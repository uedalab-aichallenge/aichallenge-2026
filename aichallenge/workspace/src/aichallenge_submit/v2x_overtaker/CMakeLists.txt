cmake_minimum_required(VERSION 3.8)
project(v2x_overtaker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(v2x_overtaker_node
  src/v2x_overtaker.cpp
)

ament_auto_package()
