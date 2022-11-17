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
    ${alica_common_config_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include src/test/Expr/include
  LIBRARIES autogen_code_supplementary_tests
  CATKIN_DEPENDS      alica_ros_proxy
  DEPENDS alica_solver_interface alica_engine alica_dummy_proxy alica_simple_solver constraintsolver alica_dynamic_loading
)

include_directories(include src/test/Expr/include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE autogen_SOURCES "src/test/Expr/src/*.cpp")
add_library(autogen_code_supplementary_tests ${autogen_SOURCES};)

target_link_libraries(autogen_code_supplementary_tests ${catkin_LIBRARIES}
                                                        Boost::system
                                                        Boost::filesystem
                                                        ${CMAKE_DL_LIBS}
                                                        ${YAML_CPP_LIBRARIES}
                                                        alica_test_utility
                                                        alica_dynamic_loading
                                                        constraintsolver
                                                        )

if (BUILD_TESTING)

    find_package(rostest REQUIRED)
    set(test_supplementary_SOURCES
        src/test_supplementary.cpp
        src/test/test_main.cpp
        src/test/test_alica_dynamicloadcreator.cpp
        src/test/test_alica_problem_composition.cpp
        src/test/test_alica_ros_proxy.cpp
        src/test/test_variable_handling.cpp
        src/test/test_alica_gsolver_plan.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}-test test/supplementary_tests.test ${test_supplementary_SOURCES})
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} autogen_code_supplementary_tests )

    add_subdirectory(alica_supplementary_test_library)

endif()