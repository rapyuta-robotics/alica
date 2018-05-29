cmake_minimum_required(VERSION 2.8.3)
project(alica_common_config)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CFG_EXTRAS cflags.cmake
)

include_directories(include include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)