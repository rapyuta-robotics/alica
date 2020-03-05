cmake_minimum_required(VERSION 3.5.1)
project(system_config)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin QUIET)
if(catkin_FOUND)
	find_package(catkin REQUIRED COMPONENTS fsystem roslib)
else(catkin_FOUND)
	set (catkin_INCLUDE_DIRS "${fsystem_INCLUDE_DIRS}")
	set (catkin_LIBRARIES "fsystem")
endif(catkin_FOUND)

if (catkin_FOUND)
  catkin_package(
   INCLUDE_DIRS include
   LIBRARIES system_config
   CATKIN_DEPENDS fsystem roslib
  )
endif(catkin_FOUND)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(system_config
  src/SystemConfig.cpp
  src/Configuration.cpp
)

target_link_libraries(system_config ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
  target_include_directories(system_config PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif(NOT catkin_FOUND)

## Add gtest based cpp test target and link libraries
if (catkin_FOUND)
  if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/test_system_config.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
    endif()
  endif(CATKIN_ENABLE_TESTING)
endif(catkin_FOUND)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
