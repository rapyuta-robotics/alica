cmake_minimum_required(VERSION 2.8.3)
project(constraintsolver)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 -ggdb ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS
  fsystem
  autodiff
  system_config
  alica_engine
  alica_ros_proxy
  roslib
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES constraintsolver
 CATKIN_DEPENDS fsystem autodiff roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(constraintsolver
  src/intervals/DownwardPropagator.cpp
  src/intervals/IntervalPropagator.cpp
  src/intervals/RecursivePropagate.cpp
  src/intervals/ResetIntervals.cpp
  src/intervals/SetParents.cpp
  src/intervals/TermList.cpp
  src/intervals/UnsolveableException.cpp
  src/intervals/UpwardPropagator.cpp
  src/types/Clause.cpp
  src/types/DecisionLevel.cpp
  src/types/Lit.cpp
  src/types/Var.cpp
  src/types/Watcher.cpp
  src/CGSolver.cpp
  src/CNSat.cpp
  src/CNSMTGSolver.cpp
  src/Decider.cpp
  src/FormulaTransform.cpp
  src/GSolver.cpp
)

target_link_libraries(constraintsolver ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-test test/test_constraintsolver.cpp test/test_cnsat.cpp test/test_cnsmtgsolver.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
