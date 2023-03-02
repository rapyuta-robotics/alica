cmake_minimum_required(VERSION 3.10)
project(alica_tests)
include(CTest)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    alica_ros_proxy
)

# #Yaml
find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)
find_package(alica_simple_solver REQUIRED)
find_package(alica_test_utility REQUIRED)
find_package(alica_dynamic_loading REQUIRED)
include_directories(include)

catkin_package(
    INCLUDE_DIRS include src/test/Expr/include
    LIBRARIES autogen_code_alica_tests
    CATKIN_DEPENDS    alica_ros_proxy
    DEPENDS alica_solver_interface alica_engine alica_dummy_proxy alica_simple_solver alica_test_utility alica_dynamic_loading
)

include_directories(include src/test/Expr/include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE autogen_SOURCES "src/test/Expr/src/*.cpp")
add_library(autogen_code_alica_tests ${autogen_SOURCES})
target_link_libraries(autogen_code_alica_tests ${catkin_LIBRARIES} alica_test_utility alica_dummy_proxy alica_dynamic_loading)

include_directories(libalica-tests/include)
file(GLOB_RECURSE AlicaTestLibSources "libalica-tests/src/*.cpp")
add_library(alica-tests SHARED ${AlicaTestLibSources} ${autogen_SOURCES})
target_link_libraries(alica-tests ${catkin_LIBRARIES} alica_test_utility alica_dummy_proxy alica_dynamic_loading)

install(TARGETS autogen_code_alica_tests alica-tests
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


if(BUILD_TESTING)
    find_package(rostest REQUIRED)

    set(test_alica_SOURCES
        src/ConstraintTestPlanDummySolver.cpp
        src/DistBallRobot.cpp
        src/DistXContourTest.cpp
        src/DummyTestSummand.cpp
        src/SimpleSwitches.cpp
        src/SwitchEntryPointsSummand.cpp
        src/TestConstantValueSummand.cpp
        src/TestWorldModel.cpp
        src/test_alica.cpp
        src/test/test_main.cpp
        src/test/test_alica_authority.cpp
        src/test/test_alica_behaviourtrigger.cpp
        src/test/test_alica_condition_plan.cpp
        src/test/test_alica_condition_plantype.cpp
        src/test/test_alica_engine_plan_parser.cpp
        src/test/test_alica_init_shutdown.cpp
        src/test/test_alica_multi_agent_plan.cpp
        src/test/test_alica_simple_plan.cpp
        src/test/test_alica_sync_transition.cpp
        src/test/test_alica_time.cpp
        src/test/test_assignment.cpp
        src/test/test_back_forth.cpp
        src/test/test_continue_on_fail.cpp
        src/test/test_success_spam.cpp
        src/test/test_task_assignment.cpp
        src/test/test_variant.cpp
        src/test/test_varsync.cpp
        src/test/test_agent_dies.cpp
        src/test/test_agent_discovery.cpp
        src/test/test_yaml_cfg.cpp
        src/test/test_path_parser.cpp
        src/test/test_config_change.cpp
        src/test/test_alica_scheduling.cpp
        src/test/test_adjacent_plans_success.cpp
        src/test/test_alica_tracing.cpp
        src/test/test_blackboard.cpp
        src/test/test_inherit_blackboard.cpp
        src/test/test_failure_handling.cpp
        src/test/test_alica_logger.cpp
        src/test/test_alica_engine_worldmodel.cpp
        src/test/test_legacy_conditions.cpp
        src/test/test_success.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}-test test/alica_tests.test ${test_alica_SOURCES})
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} alica_dynamic_loading alica-tests)
endif()
