cmake_minimum_required(VERSION 3.5.1)
project(alica_turtlesim)

# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(alica_engine REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(constraintsolver REQUIRED)

include_directories(include)

add_library(libalica-turtlesim SHARED
  libalica-turtlesim/src/CircleRuntimeCondition.cpp
  libalica-turtlesim/src/Idle.cpp
  libalica-turtlesim/src/GoTo.cpp
  libalica-turtlesim/src/GoToCalculatedResult.cpp
  libalica-turtlesim/src/MakeFormation.cpp
  libalica-turtlesim/src/RotateTurtle.cpp
  libalica-turtlesim/src/SpawnTurtle.cpp
  libalica-turtlesim/src/Teleport.cpp
)
target_link_libraries(libalica-turtlesim alica_engine)

target_include_directories(libalica-turtlesim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libalica-turtlesim/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS libalica-turtlesim
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Export targets to make package libraries
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY etc
    DESTINATION share/${PROJECT_NAME}/
)


#Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)
