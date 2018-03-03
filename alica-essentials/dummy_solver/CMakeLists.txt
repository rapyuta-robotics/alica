cmake_minimum_required(VERSION 2.8.3)
project(dummy_solver)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 -ggdb ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED alica_engine)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dummy_solver
  CATKIN_DEPENDS alica_engine
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/DummySolver.cpp
   src/DummyTerm.cpp
   src/DummyVariable.cpp
)

 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
