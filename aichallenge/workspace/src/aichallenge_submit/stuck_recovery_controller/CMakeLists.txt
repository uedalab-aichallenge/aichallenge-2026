cmake_minimum_required(VERSION 3.8)
project(stuck_recovery_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(stuck_recovery_controller_node
  src/stuck_recovery_controller.cpp
  src/recovery_planner.cpp
  src/escape_certificate.cpp
  src/reachable_steering.cpp
)

if(BUILD_TESTING)
  add_executable(test_reachable_steering
    test/test_reachable_steering.cpp src/reachable_steering.cpp src/recovery_planner.cpp)
  target_include_directories(test_reachable_steering PRIVATE include)
  target_compile_features(test_reachable_steering PRIVATE cxx_std_17)
  add_test(NAME reachable_steering_invariants COMMAND test_reachable_steering)
  add_executable(test_phase_path
    test/test_phase_path.cpp src/recovery_planner.cpp src/escape_certificate.cpp)
  target_include_directories(test_phase_path PRIVATE include)
  target_compile_features(test_phase_path PRIVATE cxx_std_17)
  add_test(NAME phase_path_invariants COMMAND test_phase_path)

  add_executable(test_escape_certificate test/test_escape_certificate.cpp src/escape_certificate.cpp)
  target_include_directories(test_escape_certificate PRIVATE include)
  target_compile_features(test_escape_certificate PRIVATE cxx_std_17)
  add_test(NAME escape_certificate_invariants COMMAND test_escape_certificate)

  add_executable(test_contact_lease test/test_contact_lease.cpp)
  target_include_directories(test_contact_lease PRIVATE include)
  target_compile_features(test_contact_lease PRIVATE cxx_std_17)
  add_test(NAME contact_lease_invariants COMMAND test_contact_lease)
endif()

ament_auto_package()
