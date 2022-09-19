cmake_minimum_required(VERSION 3.5.1)
project(supplementary_tests)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    fsystem
    alica_engine
    alica_dummy_proxy
    alica_ros_proxy
    alica_simple_solver
    alica_common_config
    alica_solver_interface
    alica_test_utility
    constraintsolver
    autodiff
    alica_ros_proxy
)

set(BOOST_MIN_VERSION "1.71.0")
find_package(Boost ${BOOST_MIN_VERSION} REQUIRED COMPONENTS filesystem system)

##Yaml
find_package(yaml-cpp REQUIRED)

include_directories(include)

catkin_package(
  INCLUDE_DIRS include src/test/Expr/include
  LIBRARIES autogen_code_supplementary_tests
  CATKIN_DEPENDS alica_engine alica_dummy_proxy alica_solver_interface alica_simple_solver constraintsolver alica_ros_proxy
)

include_directories(include src/test/Expr/include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE autogen_SOURCES "src/test/Expr/src/*.cpp")
add_library(autogen_code_supplementary_tests ${autogen_SOURCES};)
target_link_libraries(autogen_code_supplementary_tests ${catkin_LIBRARIES} 
                                                        Boost::system
                                                        Boost::filesystem
                                                        ${CMAKE_DL_LIBS}
                                                        ${YAML_CPP_LIBRARIES})

if (CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)
    #file(GLOB_RECURSE test_supplementary_SOURCES "src/test/*.cpp" src/test_supplementary.cpp)
    set(test_supplementary_SOURCES
        src/test_supplementary.cpp
        src/test/test_main.cpp
        #src/test/test_alica_dynamicloadcreator.cpp
        src/test/test_alica_problem_composition.cpp
        src/test/test_alica_ros_proxy.cpp
        src/test/test_variable_handling.cpp
        src/test/test_alica_gsolver_plan.cpp
    )

    add_rostest_gtest(${PROJECT_NAME}-test test/supplementary_tests.test ${test_supplementary_SOURCES})
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} autogen_code_supplementary_tests )

endif()
