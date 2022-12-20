cmake_minimum_required(VERSION 3.10)
project(alica_tests)
include(CTest)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)
# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)


# #Yaml
find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)
find_package(alica_simple_solver REQUIRED)
find_package(alica_test_utility REQUIRED)
include_directories(include)

include_directories(include src/test/Expr/include)
file(GLOB_RECURSE autogen_SOURCES "src/test/Expr/src/*.cpp")
add_library(alica_tests SHARED ${autogen_SOURCES})
target_include_directories(alica_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(alica_tests alica_test_utility alica_dummy_proxy)
target_compile_definitions(alica_tests PUBLIC "PLANS=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

if(BUILD_TESTING)
    find_package(GTest)
    include(GoogleTest)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)


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
        src/test/test_legacy_conditions.cpp
    )

    add_executable(${PROJECT_NAME}-test ${test_alica_SOURCES})
    target_link_libraries(${PROJECT_NAME}-test PRIVATE Threads::Threads alica_tests ${YAML_CPP_LIBRARIES} ${GTEST_LIBRARIES})
    gtest_discover_tests(${PROJECT_NAME}-test)
endif()

# Install
install(TARGETS ${PROJECT_NAME}
EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# Export targets to make package libraries
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)


# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.9.4
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)
