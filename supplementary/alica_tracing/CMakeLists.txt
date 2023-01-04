cmake_minimum_required(VERSION 3.5.1)
project(alica_tracing)
add_compile_options(-std=c++17)


include(../../cmake_flags/cflags.cmake)
alica_compile_flags()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

find_package(yaml-cpp REQUIRED)
find_package(alica_engine REQUIRED)


include_directories(
  include
  ${alica_engine_INCLUDE_DIRS}
  SYSTEM
)

###########
## Build ##
###########

# Declare a C++ library with project namespace to avoid naming collision
add_library(
  ${PROJECT_NAME} SHARED
  src/tracing/Trace.cpp
  src/tracing/TraceFactory.cpp
)

target_include_directories(alica_tracing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
  "-Wl,-rpath=/usr/local/lib/,--disable-new-dtags"
  jaegertracing
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

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
  VERSION 0.9.4
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
