cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp process_manager system_util alica_ros_proxy alica_client pm_control std_msgs message_generation msl_actuator_msgs)

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
include(${QT_USE_FILE})

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

add_message_files(
  FILES
  RobotCommand.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
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
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(robot_control_RCS
  resources/resources.qrc
)

catkin_package(
  INCLUDE_DIRS ${robot_control_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)


qt4_wrap_cpp(robot_control_MOCS ${robot_control_HDRS})
qt4_add_resources(robot_control_RESOURCES ${robot_control_RCS}) 
qt4_wrap_ui(robot_control_UIS_H ${robot_control_UIS})

include_directories(${robot_control_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${robot_control_SRCS} ${robot_control_MOCS} ${robot_control_UIS_H} ${robot_control_RESOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY})

add_dependencies(${PROJECT_NAME} ${catkin_LIBRARIES} robot_control_generate_messages_cpp)

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
