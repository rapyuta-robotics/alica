cmake_minimum_required(VERSION 2.8.3)
project(id)

add_compile_options(-std=c++11)

# Define where to find modules for UUID
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

find_package(catkin REQUIRED)
find_package(UUID)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES id
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#### ID MESSAGES
find_package(CapnProto REQUIRED)
set(${PROJECT_NAME}_message_folder "${CMAKE_CURRENT_SOURCE_DIR}/include/essentials")
file(GLOB_RECURSE capnp_messages ${${PROJECT_NAME}_message_folder} *.capnp)
set(CAPNPC_SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/include/essentials)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/essentials")
set(CAPNPC_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/essentials")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

message("id: CAPNP_SRCS is '${CAPNP_SRCS}'")

add_library(${PROJECT_NAME}
  ${CAPNP_SRCS}
  src/Identifier.cpp
  src/IDManager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${UUID_LIBRARIES}
)