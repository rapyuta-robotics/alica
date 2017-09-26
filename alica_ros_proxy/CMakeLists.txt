cmake_minimum_required(VERSION 2.8.3)
project(alica_ros_proxy)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS
  alica_engine
  system_config
  roscpp
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  DIRECTORY msg FILES
  AllocationAuthorityInfo.msg
  EntryPointRobots.msg
  RoleSwitch.msg
  SolverVar.msg
  SyncReady.msg
  AlicaEngineInfo.msg 
  PlanTreeInfo.msg
  SolverResult.msg
  SyncData.msg
  SyncTalk.msg   
  RobotID.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES alica_ros_proxy
  CATKIN_DEPENDS alica_engine roscpp message_runtime
  #DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(alica_ros_proxy
	src/clock/AlicaROSClock.cpp src/communication/AlicaRosCommunication.cpp
)

add_dependencies(alica_ros_proxy alica_ros_proxy_generate_messages_cpp)

target_link_libraries(alica_ros_proxy
  ${catkin_LIBRARIES}
)
