cmake_minimum_required(VERSION 2.8.3)
project(system_util)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

## Enable exception handling for segfaults
set(CMAKE_CXX_FLAGS "-rdynamic -g -fnon-call-exceptions -ggdb ${CMAKE_CXX_FLAGS}")

find_package(catkin QUIET)
if(catkin_FOUND)
	find_package(catkin REQUIRED COMPONENTS system_config fsystem)
else(catkin_FOUND)
	set (catkin_INCLUDE_DIRS "${system_config_INCLUDE_DIRS}")
	set (catkin_LIBRARIES "system_config fsystem")
endif(catkin_FOUND)

if (catkin_FOUND)
  catkin_package(
   # exporting include/essentials is required to avoid breaking existing code
   # TODO: fix existing code to include headers using essentials/<header> & then remove this
   INCLUDE_DIRS include include/essentials
   LIBRARIES system_util fsystem
   CATKIN_DEPENDS system_config fsystem
  # DEPENDS system filesystem
  )
endif(catkin_FOUND)

include_directories(
  include
  # TODO: see comment in catkin_package() above to remove this
  include/essentials
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/Logging.cpp
   src/ConsoleCommandHelper.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
  target_include_directories(system_util PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
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

# install headers to global include/ [required to avoid breaking existing code]
# TODO: fix existing code to include headers using essentials/<header> & then remove this
install(DIRECTORY include/essentials/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)