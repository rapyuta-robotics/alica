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
set(${PROJECT_NAME}_msgs "${CMAKE_CURRENT_SOURCE_DIR}/include/essentials")
file(GLOB_RECURSE capnp_messages ${${PROJECT_NAME}_msgs} *.capnp)
set(CAPNPC_SRC_PREFIX ${${PROJECT_NAME}_msgs})
set(CAPNPC_OUTPUT_DIR ${${PROJECT_NAME}_msgs})
#message("id: capnp_messages is '${capnp_messages}'")
capnp_generate_cpp(CAPNP_SRCS CAPNP_HDRS ${capnp_messages})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UUID_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${CAPNP_SRCS}
  src/Identifier.cpp
  src/IDManager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${UUID_LIBRARIES}
)