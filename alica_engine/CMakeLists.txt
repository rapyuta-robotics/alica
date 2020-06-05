cmake_minimum_required(VERSION 3.5.1)
project(alica_engine)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

##Yaml
find_package(yaml-cpp REQUIRED)

find_package(catkin QUIET)
if (catkin_FOUND)
  find_package(catkin REQUIRED COMPONENTS id_manager system_config fsystem event_handling system_util alica_common_config alica_solver_interface)
else (catkin_FOUND)
  set(catkin_LIBRARIES "id_manager" "system_config" "fsystem" "event_handling" "system_util" "alica_common_config" "alica_solver_interface")
endif(catkin_FOUND)

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES alica_engine
    CATKIN_DEPENDS id_manager system_config fsystem event_handling system_util alica_common_config alica_solver_interface
  )
endif(catkin_FOUND)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

file(GLOB_RECURSE alica_SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} ${alica_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

if (NOT catkin_FOUND)
 target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
