cmake_minimum_required(VERSION 3.5.1)
project(alica_ros_proxy)
include(CTest)

add_compile_options(-std=c++17)
include(../../../cmake_flags/cflags.cmake)
alica_compile_flags()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  alica_msgs
)

find_package(alica_engine REQUIRED)
find_package(alica_solver_interface REQUIRED)
## System dependencies are found with CMake's conventions

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES alica_ros_proxy
  CATKIN_DEPENDS roscpp
  DEPENDS  alica_solver_interface alica_engine
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${alica_solver_interface_INCLUDE_DIRS}
  ${alica_engine_INCLUDE_DIRS}
)

add_library(alica_ros_proxy
	src/clock/AlicaROSClock.cpp
	src/communication/AlicaRosCommunication.cpp
  src/logger/AlicaRosLogger.cpp
)

add_dependencies(alica_ros_proxy alica_msgs_generate_messages_cpp)

target_link_libraries(alica_ros_proxy
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(catkin REQUIRED COMPONENTS
    rostest
    roscpp
  )
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
  add_rostest_gtest(alica_ros_timer_tests test/alica_ros_timer_tests.test test/alica_ros_timer_tests.cpp)
  target_link_libraries(alica_ros_timer_tests ${catkin_LIBRARIES})
endif()
