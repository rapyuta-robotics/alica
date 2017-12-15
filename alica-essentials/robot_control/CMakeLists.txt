cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
    rqt_gui 
    rqt_gui_cpp 
    process_manager 
    system_util
    agent_id 
    alica_client 
    pm_control 
    std_msgs 
    message_generation 
)

#find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
find_package(Qt5Core REQUIRED)
get_target_property(Qt5Core_location Qt5::Core LOCATION)
find_package(Qt5Gui REQUIRED)
get_target_property(Qt5Gui_location Qt5::Gui LOCATION)
find_package(Qt5Network REQUIRED)
get_target_property(Qt5Network_location Qt5::Network LOCATION)
find_package(Qt5Widgets REQUIRED)
get_target_property(Qt5Widgets_location Qt5::Widgets LOCATION)
SET( QT_USE_QTXML TRUE )
SET( QT_WRAP_CPP TRUE )

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

catkin_python_setup()

add_message_files(
  FILES
  RobotCommand.msg
)

generate_messages(
  DEPENDENCIES
  agent_id
  std_msgs
)

set(robot_control_SRCS
  src/robot_control/flowlayout.cpp
  src/robot_control/RobotsControl.cpp
  src/robot_control/Robot.cpp
)

set(robot_control_HDRS
  include/robot_control/flowlayout.h
  include/robot_control/RobotsControl.h
  include/robot_control/Robot.h
)

set(robot_control_UIS
  ui/RobotsControl.ui
  ui/ControlledRobot.ui  
)

set(robot_control_INCLUDE_DIRECTORIES
  include
  ${Qt5Core_INCLUDE_DIRS} 
  ${Qt5Gui_INCLUDE_DIRS} 
  ${Qt5Network_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
include_directories(${robot_control_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

set(robot_control_RCS
  resources/resources.qrc
)

catkin_package(
  INCLUDE_DIRS ${robot_control_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp msl_actuator_msgs msl_sensor_msgs agent_id
)


QT5_WRAP_CPP(robot_control_MOCS ${robot_control_HDRS})
QT5_ADD_RESOURCES(robot_control_RESOURCES ${robot_control_RCS}) 
QT5_WRAP_UI(robot_control_UIS_H ${robot_control_UIS})

add_library(${PROJECT_NAME} ${robot_control_SRCS} ${robot_control_MOCS} ${robot_control_UIS_H} ${robot_control_RESOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Qt5Core_location} ${Qt5Gui_location} ${Qt5Network_location})

add_dependencies(${PROJECT_NAME} robot_control_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/robot_control
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/robot_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
