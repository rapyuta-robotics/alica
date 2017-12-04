cmake_minimum_required(VERSION 2.8.3)
project(world_model)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED system_config alica_engine nonstd)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES world_model
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/WorldModel.cpp
)
