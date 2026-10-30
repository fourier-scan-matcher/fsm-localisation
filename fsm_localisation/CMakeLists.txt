cmake_minimum_required(VERSION 3.16)
project(fsm_localisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FSM_LOCALISATION_EXACT_FP "Forbid floating point reassociation and contraction" OFF)
option(FSM_LOCALISATION_DETERMINISTIC_FFT "Select FFTW plans by heuristic, not by timing" OFF)

# The arithmetic, chosen here and recorded for anything that builds against
# this package. The matcher is a header a dependent package compiles for
# itself, so two packages compiling it under different arithmetic put two
# definitions of one inline function into a single process. Whoever builds this
# package therefore settles the arithmetic for everything above it, and
# cmake/fsm_localisation-extras.cmake.in is where the choice is written down.
if(FSM_LOCALISATION_EXACT_FP)
  set(FSM_LOCALISATION_BUILT_COMPILE_OPTIONS -O2 -fno-fast-math -ffp-contract=off)
else()
  set(FSM_LOCALISATION_BUILT_COMPILE_OPTIONS -Ofast -fno-finite-math-only)
endif()

add_compile_options(${FSM_LOCALISATION_BUILT_COMPILE_OPTIONS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)

add_library(fsm_localisation_core SHARED
  src/fsm_localisation/fsm_localisation.cpp)

target_include_directories(fsm_localisation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(fsm_localisation_core PUBLIC
  Eigen3::Eigen
  CGAL::CGAL
  PkgConfig::FFTW3)

if(FSM_LOCALISATION_DETERMINISTIC_FFT)
  target_compile_definitions(fsm_localisation_core PUBLIC FSM_LOCALISATION_FFTW_ESTIMATE)
endif()

add_library(fsm_localisation_grid SHARED
  src/fsm_localisation/occupancy_grid_map.cpp)

target_link_libraries(fsm_localisation_grid PUBLIC
  fsm_localisation_core
  ${nav_msgs_TARGETS})

add_library(fsm_localisation_interface SHARED
  src/fsm_localisation/fsm_localisation_interface.cpp)

target_link_libraries(fsm_localisation_interface PUBLIC
  fsm_localisation_core
  fsm_localisation_grid
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS})

add_executable(fsm_localisation_interface_node
  src/fsm_localisation/fsm_localisation_interface_node.cpp)

target_link_libraries(fsm_localisation_interface_node fsm_localisation_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/range_libc/*)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_library(fsm_localisation_core_exact OBJECT
    src/fsm_localisation/fsm_localisation.cpp)

  target_include_directories(fsm_localisation_core_exact PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(fsm_localisation_core_exact PUBLIC
    Eigen3::Eigen
    CGAL::CGAL
    PkgConfig::FFTW3)

  target_compile_options(fsm_localisation_core_exact PUBLIC
    -O2 -fno-fast-math -ffp-contract=off)

  target_compile_definitions(fsm_localisation_core_exact PUBLIC
    FSM_LOCALISATION_FFTW_ESTIMATE)

  add_library(fsm_localisation_grid_exact OBJECT
    src/fsm_localisation/occupancy_grid_map.cpp)

  target_link_libraries(fsm_localisation_grid_exact PUBLIC
    fsm_localisation_core_exact
    ${nav_msgs_TARGETS})

  ament_add_gtest(test_fsm_localisation_cbgl_defects
    test/test_cbgl_defects.cpp)

  ament_add_gtest(test_fsm_localisation_core
    test/test_defects_core.cpp)

  ament_add_gtest(test_fsm_localisation_corrector
    test/test_corrector.cpp
    TIMEOUT 120)

  ament_add_gtest(test_fsm_localisation_core_golden
    test/test_core_golden.cpp)

  ament_add_gtest(test_fsm_localisation_dft
    test/test_dft.cpp)

  ament_add_gtest(test_fsm_localisation_finite_guard
    test/test_finite_guard.cpp)
  target_include_directories(test_fsm_localisation_finite_guard PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_fsm_localisation_finite_guard
    Eigen3::Eigen
    CGAL::CGAL
    PkgConfig::FFTW3)
  target_compile_options(test_fsm_localisation_finite_guard PRIVATE
    "SHELL:-Ofast -ffast-math -fno-finite-math-only -ffp-contract=fast")
  target_compile_definitions(test_fsm_localisation_finite_guard PRIVATE
    FSM_LOCALISATION_FFTW_ESTIMATE)

  ament_add_gtest(test_fsm_localisation_geometry
    test/test_geometry.cpp)

  ament_add_gtest(test_fsm_localisation_invalid_ranges
    test/test_invalid_ranges.cpp)

  ament_add_gtest(test_fsm_localisation_liveness_guards
    test/test_liveness_guards.cpp
    TIMEOUT 10)

  ament_add_gtest(test_fsm_localisation_occupancy_grid_map
    test/test_occupancy_grid_map.cpp)

  ament_add_gtest(test_fsm_localisation_node_params
    test/test_node_params.cpp
    ENV ROS_DOMAIN_ID=71)
  target_link_libraries(test_fsm_localisation_node_params
    fsm_localisation_interface)

  ament_add_gtest(test_fsm_localisation_params
    test/test_params.cpp)

  ament_add_gtest(test_fsm_localisation_ray_casting
    test/test_ray_casting.cpp)

  ament_add_gtest(test_fsm_localisation_reference_agreement
    test/test_reference_agreement.cpp)

  ament_add_gtest(test_fsm_localisation_reference_handle
    test/test_reference_handle.cpp)

  ament_add_gtest(test_fsm_localisation_scan_to_map_switches
    test/test_scan_to_map_switches.cpp)

  ament_add_gtest(test_fsm_localisation_scan_handling
    test/test_scan_handling.cpp)

  ament_add_gtest(test_fsm_localisation_stages
    test/test_stages.cpp)

  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_node_startup.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=72
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_launch_test(test/test_node_behaviour.py TIMEOUT 400
    ENV ROS_DOMAIN_ID=73
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_initial_pose.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=74
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_amelioration_behaviour.py TIMEOUT 600
    ENV ROS_DOMAIN_ID=75
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_amelioration_startup.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=76
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_mode_switch.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=77
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_refuses_to_start.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=78
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_upside_down_sensor.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=79
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  foreach(test_target test_fsm_localisation_cbgl_defects test_fsm_localisation_core
                      test_fsm_localisation_core_golden
                      test_fsm_localisation_dft test_fsm_localisation_geometry
                      test_fsm_localisation_invalid_ranges
                      test_fsm_localisation_liveness_guards test_fsm_localisation_params
                      test_fsm_localisation_reference_handle
                      test_fsm_localisation_scan_handling
                      test_fsm_localisation_scan_to_map_switches
                      test_fsm_localisation_stages)
    if(TARGET ${test_target})
      target_link_libraries(${test_target} fsm_localisation_core_exact)
      target_compile_definitions(${test_target} PRIVATE
        FSM_LOCALISATION_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    endif()
  endforeach()

  foreach(test_target test_fsm_localisation_corrector
                      test_fsm_localisation_occupancy_grid_map
                      test_fsm_localisation_ray_casting
                      test_fsm_localisation_reference_agreement)
    if(TARGET ${test_target})
      target_link_libraries(${test_target}
        fsm_localisation_core_exact
        fsm_localisation_grid_exact)
    endif()
  endforeach()
endif()

install(TARGETS
  fsm_localisation_core
  fsm_localisation_grid
  fsm_localisation_interface
  EXPORT export_fsm_localisation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(TARGETS
  fsm_localisation_interface_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_fsm_localisation HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  CGAL)

configure_file(
  cmake/fsm_localisation-extras.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/fsm_localisation-extras.cmake
  @ONLY)

ament_package(CONFIG_EXTRAS
  ${CMAKE_CURRENT_BINARY_DIR}/fsm_localisation-extras.cmake)
