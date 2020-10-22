cmake_minimum_required(VERSION 3.5.1)
project(alica_engine)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

##Yaml
find_package(yaml-cpp REQUIRED)

##ROS/Catkin
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

include_directories(include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR})

file(GLOB_RECURSE alica_SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} ${alica_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

if (NOT catkin_FOUND)
 target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif (NOT catkin_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

#############
## Doxygen ##
#############

find_package(Doxygen)
option(BUILD_DOXYGEN "Generate Doxygen Documentation")
if (DOXYGEN_FOUND AND BUILD_DOXYGEN)
  # set input and output files
  set(DOCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../docs/doxygen)
  set(DOXYGEN_IN ${DOCS_DIR}/../Doxyfile.in)
  set(DOXYGEN_OUT ${DOCS_DIR}/${PROJECT_NAME}/Doxyfile)

  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT})
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target(${PROJECT_NAME}_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen into ${DOCS_DIR}"
    VERBATIM
  )
elseif (BUILD_DOXYGEN)
        message("Doxygen needs to be installed to generate the Doxygen Documentation for ${PROJECT_NAME}")
endif (DOXYGEN_FOUND AND BUILD_DOXYGEN)
