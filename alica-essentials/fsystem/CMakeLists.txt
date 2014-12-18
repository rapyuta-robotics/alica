cmake_minimum_required(VERSION 2.8.3)
project(fsystem)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if (catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fsystem
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )
endif (catkin_FOUND)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)

## Declare a cpp library
 add_library(fsystem
   src/FileSystem.cpp
 )

if (NOT catkin_FOUND)
  target_include_directories(fsystem PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

