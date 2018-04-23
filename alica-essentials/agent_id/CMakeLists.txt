cmake_minimum_required(VERSION 3.5.1)
project(agent_id)
 
## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Define where to find modules for UUID
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

find_package(catkin REQUIRED roscpp message_generation)
find_package(UUID)

add_message_files(
  FILES
  AgentID.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agent_id
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS UUID 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/AgentID.cpp
  src/AgentIDFactory.cpp
  src/BroadcastID.cpp
  src/AgentIDManager.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${UUID_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-tests
    src/test/AgentIDTests.cpp
  )
  target_link_libraries(${PROJECT_NAME}-tests ${PROJECT_NAME} ${GTEST_LIBRARIES})
endif()
