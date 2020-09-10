cmake_minimum_required(VERSION 3.5.1)
project(cnc_geometry)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cnc_geometry
    CATKIN_DEPENDS geometry_msgs
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(cnc_geometry
    src/CNPositionEgo.cpp
    src/CNPositionAllo.cpp

    src/CNVecEgo.cpp
    src/CNVecAllo.cpp

    src/CNPointEgo.cpp
    src/CNPointAllo.cpp
)

target_link_libraries(cnc_geometry
  ${catkin_LIBRARIES}
)
