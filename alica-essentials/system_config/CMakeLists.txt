cmake_minimum_required(VERSION 3.5.1)
project(system_config)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

## Enable exception handling for segfaults using SigFault.h
set(CMAKE_CXX_FLAGS "-rdynamic -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

find_package(catkin QUIET)
if(catkin_FOUND)
	find_package(catkin REQUIRED COMPONENTS fsystem roslib)
else(catkin_FOUND)
	set (catkin_INCLUDE_DIRS "${fsystem_INCLUDE_DIRS}")
	set (catkin_LIBRARIES "fsystem")
endif(catkin_FOUND)

if (catkin_FOUND)
  catkin_package(
   # exporting include/essentials is required to avoid breaking existing code
   # TODO: fix existing code to include headers using essentials/<header> & then remove this
   INCLUDE_DIRS include include/essentials
   LIBRARIES system_config
   CATKIN_DEPENDS fsystem roslib
  )
endif(catkin_FOUND)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  # TODO: see comment in catkin_package() above to remove this
  include/essentials
  ${catkin_INCLUDE_DIRS}
)

add_library(system_config
  src/SystemConfig.cpp
  src/Configuration.cpp
  #include/Configuration.h
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

# install headers to global include/ [required to avoid breaking existing code]
# TODO: fix existing code to include headers using essentials/<header> & then remove this
install(DIRECTORY include/essentials/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)