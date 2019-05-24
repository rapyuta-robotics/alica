cmake_minimum_required(VERSION 2.8.3)
project(id_manager)

add_compile_options(-std=c++11)

# Define where to find modules for UUID
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

find_package(catkin REQUIRED)
find_package(UUID)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES id_manager
)

include_directories(
  include
  ${UUID_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Identifier.cpp
  src/IDManager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${UUID_LIBRARIES}
)
