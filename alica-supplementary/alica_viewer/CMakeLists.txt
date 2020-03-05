cmake_minimum_required(VERSION 3.5.1)
project(alica_viewer)

##################
## Dependencies ##
##################
add_compile_options(-std=c++14 -pthread)

add_compile_options(${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS})

set(PKG_DEPS
        alica_engine
        alica_common_config
        id_ros
        alica_msgs
        alica_ros_proxy
)
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})
find_package(Qt5Widgets REQUIRED)
SET(QT_WRAP_CPP TRUE)

catkin_package(
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES alica_viewer alica_msgs
    CATKIN_DEPENDS alica_engine alica_ros_proxy alica_msgs
    #DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}" # UI header path
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

add_definitions(${Qt5Widgets_DEFINITIONS})

set(QT_CPP_HEADERS
        include/alica_viewer/alica_viewer_main_window.h
        include/alica_viewer/alica_viewer_ros_interface.h
        include/alica_viewer/graphics_view_zoom.h
)

# generate MOC sources
QT5_WRAP_CPP(MOC_SOURCES ${QT_CPP_HEADERS})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
QT5_WRAP_UI(UIS_HEADERS ui/alica_viewer_main_window.ui)
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

set(QT_CPP_SOURCES 
        src/elastic_nodes/block.cpp
        src/elastic_nodes/edge.cpp
        src/elastic_nodes/node.cpp
        src/alica_viewer_main_window.cpp
        src/alica_viewer_node.cpp
        src/alica_viewer_plan_parser.cpp
        src/alica_viewer_ros_interface.cpp
        src/graphics_view_zoom.cpp
)

add_executable(${PROJECT_NAME} 
    ${QT_CPP_SOURCES}
    ${UIS_HEADERS}
    ${MOC_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
