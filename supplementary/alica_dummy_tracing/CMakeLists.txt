cmake_minimum_required(VERSION 3.10)
project(alica_dummy_tracing)

add_compile_options(-std=c++17)

find_package(catkin QUIET)
if (catkin_FOUND)
    find_package(catkin REQUIRED COMPONENTS
      alica_engine
    )
else (catkin_FOUND)
   	set(catkin_LIBRARIES "alica_engine")
endif(catkin_FOUND)

if(catkin_FOUND)
 catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS alica_engine
  #DEPENDS
 )
endif(catkin_FOUND)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if (NOT catkin_FOUND)
 target_include_directories(alica_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)