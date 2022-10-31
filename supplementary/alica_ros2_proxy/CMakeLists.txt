cmake_minimum_required(VERSION 3.5.1)
project(alica_ros2_proxy)

add_compile_options(-std=c++17)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 

find_package(ament_cmake REQUIRED)
find_package(alica_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(alica_engine REQUIRED)
find_package(alica_solver_interface REQUIRED)
## System dependencies are found with CMake's conventions

# catkin_package(
#   INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
#   LIBRARIES alica_ros2_proxy
#   CATKIN_DEPENDS roscpp
#   DEPENDS  alica_solver_interface alica_engine 
#   #DEPENDS system_lib
# )

include_directories(
	include/${PROJECT_NAME}
	# ${catkin_INCLUDE_DIRS}
  ${alica_solver_interface_INCLUDE_DIRS}
  ${alica_engine_INCLUDE_DIRS}
)

add_library(alica_ros2_proxy SHARED
	src/ros2_clock/AlicaROSClock.cpp
	src/ros2_communication/AlicaRosCommunication.cpp
  src/ros2_logger/AlicaRosLogger.cpp
)

# add_dependencies(alica_ros2_proxy alica_msgs_generate_messages_cpp)

target_include_directories(alica_ros2_proxy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

ament_target_dependencies(alica_ros2_proxy
  rclcpp
  alica_msgs
)

ament_export_include_directories(include)
ament_export_libraries(alica_ros2_proxy)

# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS
#     rostest
#     roscpp
#   )
#   include_directories(
#     ${catkin_INCLUDE_DIRS}
#   )
#   add_rostest_gtest(alica_ros_timer_tests test/alica_ros_timer_tests.test test/alica_ros_timer_tests.cpp)
#   target_link_libraries(alica_ros_timer_tests ${catkin_LIBRARIES})
# endif()
ament_package()

export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)

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
