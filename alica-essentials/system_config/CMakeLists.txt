cmake_minimum_required(VERSION 3.5.1)
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

find_package(catkin QUIET)
if(catkin_FOUND)
	find_package(catkin REQUIRED COMPONENTS fsystem)
else(catkin_FOUND)
	set (catkin_INCLUDE_DIRS "${fsystem_INCLUDE_DIRS}")
	set (catkin_LIBRARIES "fsystem")
endif(catkin_FOUND)

if (catkin_FOUND)
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES system_config
   CATKIN_DEPENDS fsystem
  )
endif(catkin_FOUND)

# SYSTEM_CONFIG LIBRARY
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(system_config
  src/SystemConfig.cpp
  src/Configuration.cpp
  include/Configuration.h
)

target_link_libraries(system_config ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
  target_include_directories(system_config PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

# SYSTEM_CONFIG TESTS
find_package(GTest REQUIRED)
include_directories(BEFORE
  ${GTEST_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}-tests test/test_system_config.cpp)
target_link_libraries(${PROJECT_NAME}-tests ${PROJECT_NAME} pthread ${GTEST_LIBRARIES})
  
