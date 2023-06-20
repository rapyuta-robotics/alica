cmake_minimum_required(VERSION 3.5.1)
project(alica-turtlesim)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake_flags/cflags.cmake)
alica_compile_flags()

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake)

find_package(alica_engine REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(alica_solver_interface REQUIRED)
find_package(autodiff REQUIRED)
find_package(turtle_interface REQUIRED)

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/CircleRuntimeCondition.cpp
  src/Idle.cpp
  src/GoTo.cpp
  src/GoToCalculatedResult.cpp
  src/MakeFormation.cpp
  src/RotateTurtle.cpp
  src/SpawnTurtle.cpp
  src/Teleport.cpp
  src/WaitForTrigger.cpp
  src/WaitForMsg.cpp
)
target_link_libraries(${PROJECT_NAME} alica_engine constraintsolver alica_solver_interface autodiff turtle_interface)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

#################

export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
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

# write_basic_package_version_file(
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
#   VERSION 1.0.0
#   COMPATIBILITY SameMajorVersion
# )

# set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

# configure_package_config_file(
#   ${CMAKE_PROJECT_NAME}Config.cmake.in
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
#   INSTALL_DESTINATION ${ConfigPackageLocation}
#   PATH_VARS INCLUDE_INSTALL_DIR
# )

# install(FILES
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
#   ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
#   DESTINATION ${ConfigPackageLocation}
# )

# install(DIRECTORY launch etc
#     DESTINATION share/${PROJECT_NAME}
# )
