cmake_minimum_required(VERSION 2.8.3)
project(alica_common_config)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  CFG_EXTRAS
    cflags.cmake
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)