cmake_minimum_required(VERSION 2.8.3)
project(alica_ros_proxy)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  alica_engine
  roscpp
  message_generation
  alica_msgs
  alica_common_config
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES alica_ros_proxy
  CATKIN_DEPENDS alica_engine roscpp
  #DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(alica_ros_proxy
	src/clock/AlicaROSClock.cpp
	src/communication/AlicaRosCommunication.cpp
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

if(CATKIN_ENABLE_TESTING)
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