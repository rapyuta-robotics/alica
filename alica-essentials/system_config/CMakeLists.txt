cmake_minimum_required(VERSION 2.8.3)
project(system_config)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

## Enable exception handling for segfaults using SigFault.h
set(CMAKE_CXX_FLAGS "-rdynamic -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin QUIET)
if(catkin_FOUND)
	find_package(catkin REQUIRED COMPONENTS fsystem roslib)
else(catkin_FOUND)
	set (catkin_INCLUDE_DIRS "${fsystem_INCLUDE_DIRS}")
	set (catkin_LIBRARIES "fsystem")
endif(catkin_FOUND)

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
   LIBRARIES system_config
   CATKIN_DEPENDS fsystem roslib
  # DEPENDS 
  )
endif(catkin_FOUND)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(system_config
  src/SystemConfig.cpp
  src/Configuration.cpp
  #include/Configuration.h
)

#add_dependencies(system_config ${catkin_LIBRARIES})

target_link_libraries(system_config ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
  target_include_directories(system_config PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

## Add gtest based cpp test target and link libraries
if (catkin_FOUND)
  catkin_add_gtest(${PROJECT_NAME}-test test/test_system_config.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif(catkin_FOUND)