cmake_minimum_required(VERSION 2.8.3)
project(process_manager)
 
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED agent_id msl_robot_id system_config system_util roscpp message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ProcessCommand.msg
  ProcessStat.msg
  ProcessStats.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   agent_id
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_executable_meta_data
  CATKIN_DEPENDS agent_id msl_robot_id system_config system_util roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(robot_executable_meta_data
  src/RobotMetaData.cpp
  src/ExecutableMetaData.cpp
  src/RobotExecutableRegistry.cpp
)
target_link_libraries(robot_executable_meta_data ${catkin_LIBRARIES})

add_executable(process_manager 
  src/ProcessManager.cpp
  src/ManagedExecutable.cpp
  src/ManagedRobot.cpp
)

target_link_libraries(process_manager ${catkin_LIBRARIES} robot_executable_meta_data)
add_dependencies(process_manager agent_id_generate_messages_cpp process_manager_generate_messages_cpp)
