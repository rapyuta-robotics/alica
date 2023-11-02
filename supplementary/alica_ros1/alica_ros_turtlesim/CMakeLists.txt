cmake_minimum_required(VERSION 3.5.1)
project(alica_ros_turtlesim)

include(../../../cmake_flags/cflags.cmake)
alica_compile_flags()
##################
## Dependencies ##
##################

set(PKG_DEPS
    roscpp
    alica_ros_proxy
    turtlesim
    geometry_msgs
)

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

##Yaml
find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface)
find_package(alica_engine REQUIRED)
find_package(alica_turtlesim REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(autodiff REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})
find_package(alica_dynamic_loading REQUIRED)

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
    INCLUDE_DIRS include
    DEPENDS autodiff alica_dynamic_loading

)


###########
## Build ##
###########
include_directories(
    include
    ${alica_turtlesim_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    src/base_node.cpp
    src/base.cpp
    src/turtle_ros1_interfaces.cpp
)

target_link_libraries(${PROJECT_NAME}
                        Boost::system
                        Boost::filesystem
                        ${CMAKE_DL_LIBS}
                        ${catkin_LIBRARIES}
                        ${YAML_CPP_LIBRARIES}
                        alica_dynamic_loading
                        autodiff
                        constraintsolver
)

add_subdirectory(libalica-ros-utils)

##########
## Test ##
##########

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}-test test/alica_ros_turtlesim.test test/test_main.cpp)
    target_link_libraries(${PROJECT_NAME}-test
        Boost::system
        Boost::filesystem
        ${CMAKE_DL_LIBS}
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        alica_dynamic_loading
        autodiff
        constraintsolver)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch etc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
