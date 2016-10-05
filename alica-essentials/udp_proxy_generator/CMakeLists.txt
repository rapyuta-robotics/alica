cmake_minimum_required(VERSION 2.8.3)
project(udp_proxy_generator)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS fsystem)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem regex)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS fsystem
 DEPENDS system filesystem regex
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_executable(udp_proxy_generator
  src/UDPCodeGen.cpp
  src/RelayedMessage.cpp
  #src/Configuration.cpp
)

add_executable(wrapped_msg_handler_generator
  src/WrappedMessageHandlerGen.cpp
  src/WrappedMessage.cpp
  #src/Configuration.cpp
)
## Declare a cpp executable
# add_executable(system_config_node src/system_config_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes



## Specify libraries to link a library or executable target against
target_link_libraries(udp_proxy_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(wrapped_msg_handler_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
