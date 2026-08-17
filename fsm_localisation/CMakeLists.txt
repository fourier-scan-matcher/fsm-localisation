cmake_minimum_required(VERSION 3.16)
project(fsm_localisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------------------------------------------------------
# FSM_LOCALISATION_EXACT_FP=ON replaces -Ofast with optimisation settings that forbid the
# compiler from reassociating floating point arithmetic or contracting
# multiply-adds. Two compilers reassociate differently, so a reference build and
# a build under test only produce comparable numbers when this is on.
#
# FSM_LOCALISATION_DETERMINISTIC_FFT=ON makes FFTW select its plan by heuristic rather
# than by timing itself. Plan choice affects rounding, so without this the same
# binary can produce different numbers on two consecutive runs.
#
# Both default to OFF, which is the configuration this package ships with. The
# tests below force both ON regardless, since their tolerances are meaningless
# otherwise.
# ------------------------------------------------------------------------------
option(FSM_LOCALISATION_EXACT_FP "Forbid floating point reassociation and contraction" OFF)
option(FSM_LOCALISATION_DETERMINISTIC_FFT "Select FFTW plans by heuristic, not by timing" OFF)

if(FSM_LOCALISATION_EXACT_FP)
  add_compile_options(-O2 -fno-fast-math -ffp-contract=off)
else()
  # -Ofast turns on -ffast-math, which turns on -ffinite-math-only. That sub
  # flag lets the compiler assume no infinity or not-a-number value ever
  # occurs, which folds a check written to catch one into a no-op and can
  # make an equality comparison against a literal report true for a value
  # that is not a number. Disabling it costs a parity check on the rare
  # double equality comparison this package's hot loops make, and keeps
  # every other relaxation -Ofast brings: the higher optimisation level,
  # reassociation, reciprocal approximations, signed zero and trapping
  # assumptions, and flush to zero.
  add_compile_options(-Ofast -fno-finite-math-only)
endif()

# ------------------------------------------------------------------------------
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

# ------------------------------------------------------------------------------
# The core: the matcher and the odometry state it drives. No ROS anywhere in
# here, so it can be tested without a ROS installation present.
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

# ------------------------------------------------------------------------------
# The occupancy grid type. It is ROS-aware in exactly one respect, the message
# it is built from, so it links nav_msgs on top of the core rather than the
# core linking it: nothing that only wants the matcher pays for a ROS message
# it does not use.
add_library(fsm_localisation_grid SHARED
  src/fsm_localisation/occupancy_grid_map.cpp)

target_link_libraries(fsm_localisation_grid PUBLIC
  fsm_localisation_core
  ${nav_msgs_TARGETS})

# ------------------------------------------------------------------------------
# The ROS interface, and the executable that runs it.
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

# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # The two formatting linters impose a brace and wrapping style that this
  # package has never used, and the core header is 3800 lines of inherited code
  # whose arithmetic must not be disturbed. Reformatting it to satisfy a linter
  # would be the single riskiest change in the package for no behavioural gain.
  # Everything that checks something other than formatting stays on.
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # range_libc is borrowed, not ours: it carries its own upstream copyright
  # headers rather than this package's, and its known defects and inherited
  # style are corrected or left alone on a separate, deliberate schedule, not
  # by a linter's opinion. The copyright and cppcheck checks both read this one
  # variable when ament_lint_auto wires them up, so setting it once here is
  # enough to keep borrowed code out of both.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/range_libc/*)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The tests assert agreement to 1e-9 and tighter. Under fast arithmetic the
  # compiler is free to reassociate, which moves results well past that, so a
  # passing test would say nothing. Force the exact configuration here whatever
  # the library itself was configured with.
  # One source file, one test. The dimensions case used to have to read values
  # nothing had ever written, and what that reported was not stable enough to
  # trust in the same process as the other three; the constructor that read
  # them is gone, and with it the reason the two were ever split.
  ament_add_gtest(test_fsm_localisation_cbgl_defects
    test/test_cbgl_defects.cpp)

  ament_add_gtest(test_fsm_localisation_core
    test/test_defects_core.cpp)

  # A regression in the recovery refusal is a hang rather than a wrong answer,
  # so this target is bounded on wall clock time as the liveness one is.
  ament_add_gtest(test_fsm_localisation_corrector
    test/test_corrector.cpp
    TIMEOUT 120)

  ament_add_gtest(test_fsm_localisation_core_golden
    test/test_core_golden.cpp)

  ament_add_gtest(test_fsm_localisation_dft
    test/test_dft.cpp)

  # The one target in this package compiled the way the package ships. The
  # matcher core is a header, so a test that calls it directly recompiles it at
  # the test's own settings whatever the test links; every other gtest target
  # below forces the exact floating point configuration, so no line of the core
  # is ever exercised as it ships. This one covers the guards that keep a value
  # which is not a number out of the matcher, which are exactly the code whose
  # behaviour depends on the arithmetic it is compiled with.
  #
  # It is deliberately excluded from both foreach loops below, is not linked
  # against fsm_localisation_core so that its own compile flags rather than the
  # library's govern the code that runs, and pins its arithmetic on the target
  # rather than inheriting it from directory scope. Three things make the pin
  # fussier than it looks, and getting any of them wrong yields a target that
  # reads as pinned and is not:
  #
  #   * FSM_LOCALISATION_EXACT_FP=ON rewrites directory scope to -fno-fast-math
  #     and -ffp-contract=off, and directory options precede target options on
  #     the command line, so both have to be restated to be undone.
  #   * gcc applies the flags implied by an -O level before it applies any
  #     explicit flag, so a later -Ofast on its own does not undo an earlier
  #     explicit -fno-fast-math. -ffast-math has to be named outright.
  #   * cmake de-duplicates compile options, so a flag restated here that also
  #     appears at directory scope is dropped from this list rather than moved
  #     to the end of it. That silently reorders the rest, and -ffast-math
  #     landing after -fno-finite-math-only turns finite math back on. The
  #     SHELL: prefix is what keeps the four flags together and in order.
  #
  # The file fails to compile if this is ever weakened.
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

  # A regression here is a hang, not a wrong answer, so the bound is on wall
  # clock time rather than on trust that the two loops under test stay bounded.
  ament_add_gtest(test_fsm_localisation_liveness_guards
    test/test_liveness_guards.cpp
    TIMEOUT 10)

  ament_add_gtest(test_fsm_localisation_occupancy_grid_map
    test/test_occupancy_grid_map.cpp)

  # The refusals the node makes before any message arrives, exercised by
  # constructing the node rather than by launching it. It links the interface
  # and takes no arithmetic settings of its own: what it asserts is which
  # configurations start and which do not, and none of that turns on a
  # floating point result.
  ament_add_gtest(test_fsm_localisation_node_params
    test/test_node_params.cpp)
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

  # The launch runner imports a test file directly rather than through pytest,
  # so the directory holding the shared helpers has to be on the import path
  # for it. Setting it here keeps the test files free of path manipulation.
  #
  # The startup contract needs a node nothing has touched, so it launches its
  # own rather than sharing the one the behaviour tests drive.
  add_launch_test(test/test_node_startup.py TIMEOUT 300
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_launch_test(test/test_node_behaviour.py TIMEOUT 400
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  # The initial pose cases launch their own node for a different reason: the
  # service they drive holds the node until a message arrives on the initial
  # pose topic, which is not a state to hand on to a case expecting an
  # answer.
  add_launch_test(test/test_initial_pose.py TIMEOUT 300
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
      target_link_libraries(${test_target} fsm_localisation_core)
      target_compile_options(${test_target} PRIVATE
        -O2 -fno-fast-math -ffp-contract=off)
      target_compile_definitions(${test_target} PRIVATE
        FSM_LOCALISATION_FFTW_ESTIMATE
        FSM_LOCALISATION_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
    endif()
  endforeach()

  # The three grid suites link the grid library rather than the core alone,
  # since what they exercise is the type that reads an occupancy grid message.
  # That library links the core publicly, so they still see everything the
  # suites above do. They read no fixture file and touch no FFT, so they take
  # the exact arithmetic settings and nothing else.
  foreach(test_target test_fsm_localisation_occupancy_grid_map
                      test_fsm_localisation_ray_casting
                      test_fsm_localisation_reference_agreement)
    if(TARGET ${test_target})
      target_link_libraries(${test_target} fsm_localisation_grid)
      target_compile_options(${test_target} PRIVATE
        -O2 -fno-fast-math -ffp-contract=off)
    endif()
  endforeach()

  # The corrector suite links the grid library for the same reason those three
  # do, and unlike them it runs the matcher, so it does reach the transform.
  # Its plans are selected by heuristic rather than by timing, so a correction
  # repeated in a later process gives the answer this one recorded.
  if(TARGET test_fsm_localisation_corrector)
    target_link_libraries(test_fsm_localisation_corrector fsm_localisation_grid)
    target_compile_options(test_fsm_localisation_corrector PRIVATE
      -O2 -fno-fast-math -ffp-contract=off)
    target_compile_definitions(test_fsm_localisation_corrector PRIVATE
      FSM_LOCALISATION_FFTW_ESTIMATE)
  endif()
endif()

# ------------------------------------------------------------------------------
install(TARGETS
  fsm_localisation_core
  fsm_localisation_grid
  fsm_localisation_interface
  fsm_localisation_interface_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(fsm_localisation_core fsm_localisation_grid fsm_localisation_interface)

ament_package()
