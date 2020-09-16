cmake_minimum_required(VERSION 3.5.1)
project(udp_proxy_generator)

add_compile_options(-std=c++14)

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
