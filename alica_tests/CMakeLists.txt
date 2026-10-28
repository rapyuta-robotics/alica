cmake_minimum_required(VERSION 3.10)
project(alica_tests)
include(CTest)

add_compile_options(-std=c++17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake)

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

# #Yaml
find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_dummy_proxy REQUIRED)
find_package(alica_simple_solver REQUIRED)
find_package(alica_test_utility REQUIRED)
find_package(alica_dynamic_loading REQUIRED)
include_directories(include)

include_directories(include)
file(GLOB_RECURSE AlicaTestLibSources "src/behaviours/*.cpp" "src/plans/*.cpp" "src/conditions/*.cpp")
add_library(alica-tests SHARED ${AlicaTestLibSources})
target_include_directories(alica-tests PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
target_link_libraries(alica-tests alica_solver_interface alica_simple_solver alica_engine ${catkin_LIBRARIES} alica_test_utility alica_dummy_proxy alica_dynamic_loading)

install(TARGETS alica-tests
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


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
        src/test/test_alica_tracing.cpp
        src/test/test_blackboard.cpp
        src/test/test_inherit_blackboard.cpp
        src/test/test_failure_handling.cpp
        src/test/test_alica_logger.cpp
        src/test/test_success.cpp
        src/test/test_standard_library_conditions.cpp
        src/test/test_alica_parallel_execution.cpp
        src/test/test_placeholders.cpp
    )
    add_executable(${PROJECT_NAME}-test ${test_alica_SOURCES})
    target_compile_definitions(${PROJECT_NAME}-test PUBLIC "THIS_PACKAGE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
    target_link_libraries(${PROJECT_NAME}-test PRIVATE Threads::Threads alica-tests ${YAML_CPP_LIBRARIES} ${GTEST_LIBRARIES})
    # The engine dlopen()s behaviour libraries off LD_LIBRARY_PATH at runtime, so a test that builds
    # an engine throws "could not load LD_LIBRARY_PATH" without it. The `check` target injects it, but
    # a bare `ctest` in the build directory does not - so set it on the tests themselves too, and
    # plain `ctest` works as expected. Keep in sync with _alica_ld_path in the top-level CMakeLists.
    if(DEFINED ENV{LD_LIBRARY_PATH} AND NOT "$ENV{LD_LIBRARY_PATH}" STREQUAL "")
        set(_alica_tests_ld_path "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}:$ENV{LD_LIBRARY_PATH}")
    else()
        set(_alica_tests_ld_path "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
    endif()
    gtest_discover_tests(${PROJECT_NAME}-test
        PROPERTIES 
        ENVIRONMENT "LD_LIBRARY_PATH=${_alica_tests_ld_path}"
        TIMEOUT 10)
endif()

# Install
install(TARGETS alica-tests
EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Export targets to make package libraries
export(EXPORT ${PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)


# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)
