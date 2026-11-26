cmake_minimum_required(VERSION 3.5.1)
project(supplementary_tests)
include(CTest)

add_compile_options(-std=c++17)

include(../../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    alica_ros_proxy
)

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

find_package(alica_solver_interface REQUIRED)
find_package(autodiff REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)
find_package(alica_simple_solver REQUIRED)
find_package(alica_test_utility REQUIRED)
find_package(alica_dynamic_loading REQUIRED)


include_directories(include
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES autogen_code_supplementary_tests
  CATKIN_DEPENDS      alica_ros_proxy
  DEPENDS alica_solver_interface alica_engine alica_dummy_proxy alica_simple_solver constraintsolver alica_dynamic_loading
)

include_directories(libalica-supplementary-tests/include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE AlicaSupplementaryTestLibSources "libalica-supplementary-tests/src/*.cpp")
add_library(supplementary-tests SHARED ${AlicaSupplementaryTestLibSources})
target_link_libraries(supplementary-tests ${catkin_LIBRARIES} alica_test_utility alica_dummy_proxy constraintsolver)

install(TARGETS supplementary-tests
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if (BUILD_TESTING)

    find_package(rostest REQUIRED)
    set(test_supplementary_SOURCES
        src/ConstraintTestPlanDummySolver.cpp
        src/test_supplementary.cpp
        src/test/test_main.cpp
        src/test/test_alica_dynamicloadcreator.cpp
        src/test/test_alica_problem_composition.cpp
        src/test/test_alica_ros_proxy.cpp
        src/test/test_variable_handling.cpp
        src/test/test_alica_gsolver_plan.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}-test test/supplementary_tests.test ${test_supplementary_SOURCES})
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} supplementary-tests alica_dynamic_loading)

    # Separate from the main suite: these tests measure timing, so they should not share a process or a time limit with it
    add_rostest_gtest(${PROJECT_NAME}-engine-timer-test test/engine_timer_factory.test
        src/ConstraintTestPlanDummySolver.cpp
        src/test_supplementary.cpp
        src/test/test_main.cpp
        src/test/test_engine_timer_factory.cpp
    )
    # No test code here uses supplementary-tests, so force the link to keep the plan library loaded until exit, as in the main
    # suite. Otherwise ~AlicaEngine unloads it with the behaviour creator and then destroys the plan's constraints, which live in it.
    target_link_libraries(${PROJECT_NAME}-engine-timer-test ${catkin_LIBRARIES} -Wl,--no-as-needed supplementary-tests -Wl,--as-needed alica_dynamic_loading)

endif()
