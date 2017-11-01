cmake_minimum_required(VERSION 2.8.3)
project(alica_client)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp alica_ros_proxy msl_actuator_msgs msl_sensor_msgs)
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

set(alica_client_INCLUDE_DIRECTORIES
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories(${alica_client_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS ${alica_client_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME} AlicaWidget
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp alica_ros_proxy msl_actuator_msgs msl_sensor_msgs
)
catkin_python_setup()

# ALICA CLIENT
set(alica_client_HDRS
  include/alica_client/AlicaClient.h
)

QT5_WRAP_CPP(alica_client_MOCS ${alica_client_HDRS})

add_library(${PROJECT_NAME} src/alica_client/AlicaClient.cpp ${alica_client_MOCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} AlicaWidget)
add_dependencies(${PROJECT_NAME} AlicaWidget)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

# ALICA WIDGET
set(alica_UIS
  src/alica/AlicaWidget.ui
)
QT5_WRAP_UI(alica_client_UIS_H ${alica_UIS})

add_library(AlicaWidget src/alica/AlicaWidget.cpp  ${alica_client_UIS_H})

target_link_libraries(AlicaWidget ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY} ${QT_QTWIDGETS_LIBRARY})
add_dependencies(AlicaWidget msl_actuator_msgs_generate_messages_cpp msl_sensor_msgs_generate_messages_cpp)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/alica_client
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/alica_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
