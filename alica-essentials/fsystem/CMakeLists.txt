cmake_minimum_required(VERSION 3.5.1)
project(fsystem)

add_compile_options(-std=c++14)

find_package(catkin QUIET)

if (catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES fsystem
  )
endif (catkin_FOUND)


include_directories(include)

add_library(fsystem
  src/FileSystem.cpp
)

if (NOT catkin_FOUND)
  target_include_directories(fsystem PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
