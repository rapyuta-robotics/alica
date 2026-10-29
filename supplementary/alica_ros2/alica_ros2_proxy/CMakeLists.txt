cmake_minimum_required(VERSION 3.5.1)
project(alica_ros2_proxy)

include(../../../cmake_flags/cflags.cmake)
alica_compile_flags()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake)

find_package(ament_cmake REQUIRED)
find_package(alica_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_solver_interface REQUIRED)

include_directories(
  include/${PROJECT_NAME}
  ${alica_solver_interface_INCLUDE_DIRS}
  ${alica_engine_INCLUDE_DIRS}
)

add_library(alica_ros2_proxy SHARED
	src/ros2_clock/AlicaROSClock.cpp
	src/ros2_communication/AlicaRosCommunication.cpp
  src/ros2_logger/AlicaRosLogger.cpp
)

target_include_directories(alica_ros2_proxy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# The engine's headers include yaml-cpp's, so the proxy needs the engine's
# usage requirements, not just its include directory -- a yaml-cpp outside the
# default include path (e.g. from Conan) is otherwise not found. Keyword-less,
# because ament_target_dependencies() below is, and CMake rejects mixing the two.
target_link_libraries(alica_ros2_proxy ${alica_engine_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_target_dependencies(alica_ros2_proxy
  rclcpp
  alica_msgs
)

ament_export_include_directories(include)
ament_export_libraries(alica_ros2_proxy)

ament_package()

# Build-tree export, for standalone (colcon) builds only. Inside the ALICA
# top-level tree, alica_msgs' targets are real targets in rosidl's install-only
# export sets rather than imported ones, which export() cannot reference.
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  export(EXPORT ${PROJECT_NAME}Targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
  )
endif()

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)
