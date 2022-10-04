cmake_minimum_required(VERSION 3.10)
project(alica_dummy_proxy)

add_compile_options(-std=c++17)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)



# find_package(catkin QUIET)
# if (catkin_FOUND)
    # find_package(catkin REQUIRED)
    find_package(alica_common_config REQUIRED)
    find_package(fsystem REQUIRED)
    find_package(alica_solver_interface REQUIRED)
    find_package(alica_engine REQUIRED)
# else (catkin_FOUND)
#    	set(catkin_LIBRARIES "alica_engine" alica_common_config)
# endif(catkin_FOUND)

# if(catkin_FOUND)
#  catkin_package(
#   INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
#   LIBRARIES alica_dummy_proxy
#   DEPENDS alica_engine alica_solver_interface
#   #DEPENDS
#  )
# endif(catkin_FOUND)

include_directories(
  include
  #${catkin_INCLUDE_DIRS}
  ${alica_common_config_INCLUDE_DIRS}
  ${alica_solver_interface_INCLUDE_DIRS}
  ${alica_engine_INCLUDE_DIRS}

)
file(GLOB_RECURSE alica_dummy_sources "src/*.cpp")
add_library(alica_dummy_proxy SHARED ${alica_dummy_sources})

# if (NOT catkin_FOUND)
 target_include_directories(alica_dummy_proxy PUBLIC 
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>)
# endif (NOT catkin_FOUND)
target_link_libraries(alica_dummy_proxy alica_engine)


install(TARGETS ${PROJECT_NAME}
EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
  )
# exported targets
install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation})


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