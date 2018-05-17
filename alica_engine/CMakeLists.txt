cmake_minimum_required(VERSION 2.8.3)
project(alica_engine)

## Change to use clang ... unfortunately it doesnt work :(
#set(CMAKE_CXX_COMPILER "clang")
#
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11  ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

## Enable rtti
set(CMAKE_CXX_FLAGS "-rdynamic ${CMAKE_CXX_FLAGS}")

find_package(catkin QUIET)
if (catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS agent_id system_config fsystem event_handling system_util alica_common_config)
else (catkin_FOUND)
  set(catkin_LIBRARIES "agent_id" "system_config" "fsystem" "event_handling" "system_util" "alica_common_config")
endif(catkin_FOUND)

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES alica_engine
    CATKIN_DEPENDS agent_id system_config fsystem event_handling system_util
  #  DEPENDS tinyxml
  )
endif(catkin_FOUND)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE alica_SOURCES "src/*.cpp")


add_library(alica_engine
    ${alica_SOURCES};
)
target_link_libraries(alica_engine ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
 target_include_directories(alica_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include") 
endif (NOT catkin_FOUND)
