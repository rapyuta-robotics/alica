cmake_minimum_required(VERSION 2.8.3)
project(fsystem)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin QUIET)

if (catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fsystem
  )
endif (catkin_FOUND)


include_directories(include)

add_library(fsystem
  src/FileSystem.cpp
)

if (NOT catkin_FOUND)
  target_include_directories(fsystem PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

