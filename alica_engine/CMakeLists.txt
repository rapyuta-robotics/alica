cmake_minimum_required(VERSION 3.5.1)
project(alica_engine)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 


##Yaml
find_package(yaml-cpp REQUIRED)

# Imports
find_package(fsystem REQUIRED)
find_package(alica_common_config REQUIRED)
find_package(alica_solver_interface REQUIRED)


###########
## Build ##
###########

include_directories(include 
  ${fsystem_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDE_DIR}
  ${alica_common_config_INCLUDE_DIRS}
  ${alica_solver_interface_INCLUDE_DIRS}
)

file(GLOB_RECURSE alica_SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${alica_SOURCES})

target_sources(alica_engine PRIVATE
  ${alica_SOURCES}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
  ${YAML_CPP_LIBRARIES} 
  fsystem 
  alica_solver_interface
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/ 
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


# Export targets to make package libraries
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)

# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.9.1
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)


#############
## Doxygen ##
#############

# The BUILD_DOXYGEN option can be set via
# catkin build <package_name> --cmake-args -DBUILD_DOXYGEN=ON
option(BUILD_DOXYGEN "Generate Doxygen Documentation")

find_package(Doxygen QUIET)
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
