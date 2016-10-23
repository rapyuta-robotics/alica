cmake_minimum_required(VERSION 2.8.3)
project(pm_control)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp process_manager system_util)

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include(${QT_USE_FILE})

set(pm_control_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
include_directories(${pm_control_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${pm_control_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME} PMWidget
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

# PROCESS MANAGER CONTROL
set(pm_control_SRCS
  src/pm_control/PMControl.cpp
)

set(pm_control_HDRS
  include/pm_control/PMControl.h
)

set(pm_control_UIS
  src/pm_control/PMControl.ui
)

qt4_wrap_cpp(pm_control_MOCS ${pm_control_HDRS})
qt4_wrap_ui(pm_control_UIS_H ${pm_control_UIS})


add_library(${PROJECT_NAME} ${pm_control_SRCS} ${pm_control_MOCS} ${pm_control_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY} PMWidget)
add_dependencies(${PROJECT_NAME} process_manager_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

# PROCESS MANAGER CONTROL WIDGET
set(pm_widget_UIS
  src/pm_widget/ProcessWidget.ui
  src/pm_widget/RobotProcessesWidget.ui
)

set(pm_widget_SRCS
  src/pm_widget/ControlledExecutable.cpp
  src/pm_widget/ControlledProcessManager.cpp
  src/pm_widget/ControlledRobot.cpp
)

set(pm_widget_HDRS
  include/pm_widget/ControlledExecutable.h
  include/pm_widget/ControlledProcessManager.h
  include/pm_widget/ControlledRobot.h
)

qt4_wrap_cpp(pm_widget_MOCS ${pm_widget_HDRS})
qt4_wrap_ui(pm_widget_UIS_H ${pm_widget_UIS})

add_library(PMWidget ${pm_widget_SRCS} ${pm_widget_MOCS} ${pm_widget_UIS_H})
target_link_libraries(PMWidget ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY})
add_dependencies(PMWidget process_manager_generate_messages_cpp)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/pm_control
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/pm_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

