cmake_minimum_required(VERSION 3.5.1)
project(alica_ros2_supplementary_tests)

include(../../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alica_ros2_proxy REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(autodiff REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)
find_package(alica_simple_solver REQUIRED)
find_package(alica_test_utility REQUIRED)
find_package(alica_dynamic_loading REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

# The tests, the plans and the test library are those of the ROS1 package. Only the fixtures and main() differ.
set(SUPPLEMENTARY_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../alica_ros1/supplementary_tests)

if(BUILD_TESTING)
  # The plans load their behaviours and conditions from this library by its name, so the name is the same as in ROS1
  file(GLOB_RECURSE AlicaSupplementaryTestLibSources "${SUPPLEMENTARY_TESTS_DIR}/libalica-supplementary-tests/src/*.cpp")
  add_library(supplementary-tests SHARED ${AlicaSupplementaryTestLibSources})
  target_include_directories(supplementary-tests PUBLIC ${SUPPLEMENTARY_TESTS_DIR}/libalica-supplementary-tests/include)
  target_link_libraries(supplementary-tests alica_test_utility alica_dummy_proxy constraintsolver)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros QUIET)

  set(test_supplementary_SOURCES
    src/test_main.cpp
    ${SUPPLEMENTARY_TESTS_DIR}/src/ConstraintTestPlanDummySolver.cpp
    ${SUPPLEMENTARY_TESTS_DIR}/src/test_supplementary.cpp
    ${SUPPLEMENTARY_TESTS_DIR}/src/test/test_alica_dynamicloadcreator.cpp
    ${SUPPLEMENTARY_TESTS_DIR}/src/test/test_alica_problem_composition.cpp
    ${SUPPLEMENTARY_TESTS_DIR}/src/test/test_alica_ros_proxy.cpp
    ${SUPPLEMENTARY_TESTS_DIR}/src/test/test_variable_handling.cpp
    ${SUPPLEMENTARY_TESTS_DIR}/src/test/test_alica_gsolver_plan.cpp
  )

  # The dynamic loading looks for libsupplementary-tests.so in LD_LIBRARY_PATH.
  # With ament_cmake_ros the test gets its own ROS domain, so tests that run in parallel do not receive its messages.
  if(COMMAND ament_add_ros_isolated_gtest)
    ament_add_ros_isolated_gtest(${PROJECT_NAME}-test ${test_supplementary_SOURCES} TIMEOUT 300 APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR})
  else()
    ament_add_gtest(${PROJECT_NAME}-test ${test_supplementary_SOURCES} TIMEOUT 300 APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR})
  endif()
  # include/ of this package first: its test_supplementary.h replaces the ROS1 header of the same name
  target_include_directories(${PROJECT_NAME}-test PRIVATE include ${SUPPLEMENTARY_TESTS_DIR}/include)
  target_compile_definitions(${PROJECT_NAME}-test PRIVATE SUPPLEMENTARY_TESTS_ROOT_PATH="${SUPPLEMENTARY_TESTS_DIR}/src/test")
  target_link_libraries(${PROJECT_NAME}-test
    supplementary-tests
    alica_ros2_proxy
    alica_dynamic_loading
    alica_simple_solver
    Boost::filesystem
  )
endif()

ament_package()
