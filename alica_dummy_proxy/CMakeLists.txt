cmake_minimum_required(VERSION 2.8.3)
project(alica_dummy_proxy)

add_compile_options(-std=c++17)

find_package(catkin QUIET)
if (catkin_FOUND)
    find_package(catkin REQUIRED COMPONENTS
      alica_engine
      alica_common_config
    )
else (catkin_FOUND)
   	set(catkin_LIBRARIES "alica_engine" alica_common_config)
endif(catkin_FOUND)

if(catkin_FOUND)
 catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES alica_dummy_proxy
  CATKIN_DEPENDS alica_engine
  #DEPENDS
 )
endif(catkin_FOUND)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
file(GLOB_RECURSE alica_dummy_sources "src/*.cpp")
add_library(alica_dummy_proxy ${alica_dummy_sources})
target_link_libraries(alica_dummy_proxy ${catkin_LIBRARIES})

if (NOT catkin_FOUND)
 target_include_directories(alica_dummy_proxy PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)