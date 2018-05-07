cmake_minimum_required(VERSION 2.8.3)
project(event_handling)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin QUIET)

if(catkin_FOUND)
	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES event_handling
	#  CATKIN_DEPENDS other_catkin_pkg
	#  DEPENDS system_lib
	)
endif(catkin_FOUND)

include_directories(include)

add_library(event_handling
   src/AutoResetEvent.cpp
   src/Timer.cpp
   src/EventTrigger.cpp
   src/Worker.cpp
)

if (NOT catkin_FOUND)
  target_include_directories(event_handling PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

if(catkin_FOUND)
 catkin_add_gtest(${PROJECT_NAME}-test test/test_event_handling.cpp)
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 endif()
endif(catkin_FOUND)