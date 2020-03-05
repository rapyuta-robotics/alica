cmake_minimum_required(VERSION 2.8.3)
project(world_model)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED system_config alica_engine nonstd id_ros)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES world_model
  CATKIN_DEPENDS system_config alica_engine nonstd id_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/WorldModel.cpp
)
