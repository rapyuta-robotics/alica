cmake_minimum_required(VERSION 3.5.1)
project(system_util)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin QUIET)
if(catkin_FOUND)
	find_package(catkin REQUIRED COMPONENTS system_config fsystem)
else(catkin_FOUND)
	set (catkin_INCLUDE_DIRS "${system_config_INCLUDE_DIRS}")
	set (catkin_LIBRARIES "system_config fsystem")
endif(catkin_FOUND)

if (catkin_FOUND)
  catkin_package(
   LIBRARIES system_util fsystem
   CATKIN_DEPENDS system_config fsystem
  # DEPENDS system filesystem
  )
endif(catkin_FOUND)

include_directories(
  include
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
