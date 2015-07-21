cmake_minimum_required(VERSION 2.8.3)
project(rqt_robot_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp process_manager system_util alica_ros_proxy)

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
include(${QT_USE_FILE})

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
#find_package(Qt5Gui REQUIRED)
#find_package(Qt5Core REQUIRED)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")



set(rqt_robot_control_SRCS
  src/rqt_robot_control/flowlayout.cpp
  src/rqt_robot_control/RobotsControl.cpp
  src/rqt_robot_control/ControlledRobot.cpp
)


set(rqt_robot_control_HDRS
  include/rqt_robot_control/flowlayout.h
  include/rqt_robot_control/RobotsControl.h
  include/rqt_robot_control/ControlledRobot.h
)

set(rqt_robot_control_UIS
  ui/RobotsControl.ui
  ui/ControlledRobot.ui  
)

set(rqt_robot_control_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  #${Qt5Gui_INCLUDE_DIRS}
  #${Qt5Core_INCLUDE_DIRS}
)

set(rqt_robot_control_RCS
  resources/resources.qrc
)

catkin_package(
  INCLUDE_DIRS ${rqt_robot_control_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(rqt_robot_control_MOCS ${rqt_robot_control_HDRS})
qt4_add_resources(rqt_robot_control_RESOURCES ${rqt_robot_control_RCS}) 
qt4_wrap_ui(rqt_robot_control_UIS_H ${rqt_robot_control_UIS})

#qt5_wrap_cpp(rqt_robot_control_MOCS ${rqt_robot_control_HDRS})
#qt5_add_resources(rqt_robot_control_RESOURCES ${rqt_robot_control_RCS}) 
#qt5_wrap_ui(rqt_robot_control_UIS_H ${rqt_robot_control_UIS})

include_directories(${rqt_robot_control_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_robot_control_SRCS} ${rqt_robot_control_MOCS} ${rqt_robot_control_UIS_H} ${rqt_robot_control_RESOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY})
add_dependencies(${PROJECT_NAME} process_manager)

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

install(PROGRAMS scripts/rqt_robot_control
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_robot_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
