cmake_minimum_required(VERSION 3.5.1)
project(fsystem)

add_compile_options(-std=c++14)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)
# find_package(catkin QUIET)

# if (catkin_FOUND)
#   catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES fsystem
#   )
# endif (catkin_FOUND)


include_directories(include)

add_library(fsystem SHARED
  src/FileSystem.cpp
)

# if (NOT catkin_FOUND)
  target_include_directories(fsystem PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# endif(NOT catkin_FOUND)
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
)

install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.0.0
  COMPATIBILITY SameMajorVersion)

  set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR)

  install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation})