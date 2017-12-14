cmake_minimum_required(VERSION 2.8.3)
project(cnc_geometry)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
    set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
    set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
    add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    msl_robot_id
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cnc_geometry
    CATKIN_DEPENDS geometry_msgs msl_robot_id
    # DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(cnc_geometry
    #src/CNPosition.cpp
    #src/CNPoint2D.cpp
    #src/CNPoint3D.cpp
    #src/CNVelocity2D.cpp

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
