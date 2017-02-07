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
  system_util
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnc_geometry
  CATKIN_DEPENDS geometry_msgs system_util
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(cnc_geometry
  src/GeometryCalculator.cpp
  #src/container/CNPosition.cpp
  #src/container/CNPoint2D.cpp
  #src/container/CNPoint3D.cpp
  #src/container/CNVelocity2D.cpp
  src/container/CNPositionEgo.cpp
  src/container/CNPositionAllo.cpp
  src/container/CNVec2DEgo.cpp
  src/container/CNVec2DAllo.cpp
  src/container/CNPoint2DEgo.cpp
  src/container/CNPoint2DAllo.cpp
  src/container/CNRobot.cpp
)

target_link_libraries(cnc_geometry
  ${catkin_LIBRARIES}
)