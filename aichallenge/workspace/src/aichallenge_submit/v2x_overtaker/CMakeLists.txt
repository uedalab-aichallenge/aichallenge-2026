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

if(BUILD_TESTING)
  add_executable(test_lateral_interval test/test_lateral_interval.cpp)
  target_include_directories(test_lateral_interval PRIVATE include)
  target_compile_features(test_lateral_interval PRIVATE cxx_std_17)
  add_test(NAME lateral_interval_invariants COMMAND test_lateral_interval)

  add_executable(test_trajectory_speed_cap test/test_trajectory_speed_cap.cpp)
  target_include_directories(test_trajectory_speed_cap PRIVATE include)
  target_compile_features(test_trajectory_speed_cap PRIVATE cxx_std_17)
  add_test(NAME trajectory_speed_cap_invariants COMMAND test_trajectory_speed_cap)

  add_executable(test_stop_nopass_latch test/test_stop_nopass_latch.cpp)
  target_include_directories(test_stop_nopass_latch PRIVATE include)
  target_compile_features(test_stop_nopass_latch PRIVATE cxx_std_17)
  add_test(NAME stop_nopass_latch_invariants COMMAND test_stop_nopass_latch)

  add_executable(test_stopped_pass_reachability test/test_stopped_pass_reachability.cpp)
  target_include_directories(test_stopped_pass_reachability PRIVATE include)
  target_compile_features(test_stopped_pass_reachability PRIVATE cxx_std_17)
  add_test(NAME stopped_pass_reachability_invariants COMMAND test_stopped_pass_reachability)

  add_executable(test_overtake_start test/test_overtake_start.cpp)
  target_include_directories(test_overtake_start PRIVATE include)
  target_compile_features(test_overtake_start PRIVATE cxx_std_17)
  add_test(NAME overtake_start_invariants COMMAND test_overtake_start)

  add_executable(test_alongside_guard test/test_alongside_guard.cpp)
  target_include_directories(test_alongside_guard PRIVATE include)
  target_compile_features(test_alongside_guard PRIVATE cxx_std_17)
  add_test(NAME alongside_guard_invariants COMMAND test_alongside_guard)

  add_executable(test_lateral_lag test/test_lateral_lag.cpp)
  target_include_directories(test_lateral_lag PRIVATE include)
  target_compile_features(test_lateral_lag PRIVATE cxx_std_17)
  add_test(NAME lateral_lag_invariants COMMAND test_lateral_lag)

  add_executable(test_pass_completion_side test/test_pass_completion_side.cpp)
  target_include_directories(test_pass_completion_side PRIVATE include)
  target_compile_features(test_pass_completion_side PRIVATE cxx_std_17)
  add_test(NAME pass_completion_side_invariants COMMAND test_pass_completion_side)

  add_executable(test_corridor_funnel test/test_corridor_funnel.cpp)
  target_include_directories(test_corridor_funnel PRIVATE include)
  target_compile_features(test_corridor_funnel PRIVATE cxx_std_17)
  add_test(NAME corridor_funnel_invariants COMMAND test_corridor_funnel)

  add_executable(test_rear_end_inpath test/test_rear_end_inpath.cpp)
  target_include_directories(test_rear_end_inpath PRIVATE include)
  target_compile_features(test_rear_end_inpath PRIVATE cxx_std_17)
  add_test(NAME rear_end_inpath_invariants COMMAND test_rear_end_inpath)

  add_executable(test_rear_end_release test/test_rear_end_release.cpp)
  target_include_directories(test_rear_end_release PRIVATE include)
  target_compile_features(test_rear_end_release PRIVATE cxx_std_17)
  add_test(NAME rear_end_release_invariants COMMAND test_rear_end_release)

  add_executable(test_launch_gate test/test_launch_gate.cpp)
  target_include_directories(test_launch_gate PRIVATE include)
  target_compile_features(test_launch_gate PRIVATE cxx_std_17)
  add_test(NAME launch_gate_invariants COMMAND test_launch_gate)
endif()

ament_auto_package()
