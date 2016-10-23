cmake_minimum_required(VERSION 2.8.3)
project(alica_dummy_proxy)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
set(CMAKE_CXX_FLAGS "-rdynamic -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

find_package(catkin QUIET)
if (catkin_FOUND)
	find_package(catkin REQUIRED COMPONENTS
	  alica_engine
	  system_config
	)
else (catkin_FOUND)
   	set(catkin_LIBRARIES "system_config" "alica_engine")
endif(catkin_FOUND)

if(catkin_FOUND)
 catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES alica_dummy_proxy
  CATKIN_DEPENDS alica_engine system_config
  #DEPENDS 
 )
endif(catkin_FOUND)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
file(GLOB_RECURSE alica_dummy_sources "src/*.cpp")
add_library(alica_dummy_proxy ${alica_dummy_sources})
target_link_libraries(alica_dummy_proxy ${catkin_LIBRARIES})
 
if (NOT catkin_FOUND)
 target_include_directories(alica_dummy_proxy PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include") 
endif (NOT catkin_FOUND)
