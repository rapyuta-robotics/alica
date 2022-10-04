cmake_minimum_required(VERSION 3.10)
project(alica_dummy_tracing)

add_compile_options(-std=c++17)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)





# find_package(catkin QUIET)
# if (catkin_FOUND)
#     find_package(catkin REQUIRED )
    find_package(alica_engine REQUIRED)
# else (catkin_FOUND)
#    	set(catkin_LIBRARIES "alica_engine")
# endif(catkin_FOUND)

# if(catkin_FOUND)
#  catkin_package(
#   INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
#   DEPENDS alica_engine
#   #DEPENDS
#  )
# endif(catkin_FOUND)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${alica_engine_INCLUDE_DIRS}
)

# if (NOT catkin_FOUND)
#  target_include_directories(alica_engine PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# endif (NOT catkin_FOUND)

install(DIRECTORY include/${PROJECT_NAME}/
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