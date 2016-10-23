cmake_minimum_required(VERSION 2.8.3)
project(udp_proxy_generator)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

find_package(catkin REQUIRED COMPONENTS fsystem)

find_package(Boost REQUIRED COMPONENTS system filesystem regex)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
 CATKIN_DEPENDS fsystem
 DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(udp_proxy_generator
  src/UDPCodeGen.cpp
  src/RelayedMessage.cpp
)

add_executable(wrapped_msg_handler_generator
  src/WrappedMessageHandlerGen.cpp
  src/WrappedMessage.cpp
)

target_link_libraries(udp_proxy_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(wrapped_msg_handler_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
