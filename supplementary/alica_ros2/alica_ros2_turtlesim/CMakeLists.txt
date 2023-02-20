cmake_minimum_required(VERSION 3.5.1)
project(alica_ros2_turtlesim)

include(../../../cmake_flags/cflags.cmake)
alica_compile_flags()

##################
## Dependencies ##
##################

##Yaml
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(alica_ros2_proxy REQUIRED)
find_package(turtlesim REQUIRED)
find_package(alica_engine REQUIRED)
find_package(constraintsolver)
find_package(autodiff REQUIRED)
find_package(alica_solver_interface)
find_package(ament_cmake REQUIRED)
find_package(alica_msgs REQUIRED)
find_package(alica_dynamic_loading REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

###########
## Build ##
###########

include_directories(
    include
    ${turtlesim_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/turtle.cpp
    src/base_node.cpp
    src/base.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${YAML_CPP_LIBRARIES}
    constraintsolver
    autodiff
    alica_solver_interface
    alica_ros2_proxy
    alica_dynamic_loading
    Boost::system
    Boost::filesystem
    ${turtlesim_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
)

ament_target_dependencies(alica_ros2_turtlesim
  rclcpp
  geometry_msgs
  alica_ros2_proxy
)

ament_export_dependencies(alica_ros2_turtlesim
    turtlesim
    geometry_msgs
    rclcpp
    alica_ros2_proxy
)

add_subdirectory(libalica-ros2-turtlesim)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch etc
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
