cmake_minimum_required(VERSION 2.8.3)
project(alica_viewer)

##################
## Dependencies ##
##################
add_compile_options(-std=c++11)
add_compile_options(${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS})

set(PKG_DEPS
        alica_engine
        alica_common_config
        agent_id
        capnzero
        alica_capnzero_proxy
)
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})
find_package(Qt5Widgets REQUIRED)
find_package(CapnProto REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES alica_viewer alica_capnz_msgs
    CATKIN_DEPENDS alica_engine capnzero alica_capnz_msgs
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
        include/alica_viewer/alica_viewer_capnzero_interface.h
        include/alica_viewer/alica_viewer_main_window.h
        include/alica_viewer/graphics_view_zoom.h
)

# generate MOC sources
qt5_wrap_cpp(MOC_SOURCES ${QT_CPP_HEADERS})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
qt5_wrap_ui(UIS_HEADERS ui/alica_viewer_main_window.ui)
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

set(QT_CPP_SOURCES 
        src/elastic_nodes/block.cpp
        src/elastic_nodes/edge.cpp
        src/elastic_nodes/node.cpp
#        src/alica_viewer_capnzero_interface.cpp
#        src/alica_viewer_main_window.cpp
#        src/alica_viewer_node.cpp
        src/alica_viewer_plan_parser.cpp
        src/graphics_view_zoom.cpp
)

# Trying to fix linker errors by making a separate target for alica_viewer_capnzeri_interface
add_library(avci src/alica_viewer_capnzero_interface.cpp)
#target_link_libraries(avci include/alica_viewer/alica_viewer_capnzero_interface.h)
target_compile_definitions(avci PUBLIC ZMQ_BUILD_DRAFT_API=1)
add_library(avmw src/alica_viewer_main_window.cpp)
target_compile_definitions(avmw PUBLIC ZMQ_BUILD_DRAFT_API=1)
add_library(avn src/alica_viewer_node.cpp)
target_compile_definitions(avn PUBLIC ZMQ_BUILD_DRAFT_API=1)


# End of try

add_executable(${PROJECT_NAME} 
    ${QT_CPP_SOURCES}
    ${UIS_HEADERS}
    ${MOC_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${CAPNP_LIBRARIES}
    avci
    avmw
    avn
)

target_compile_definitions(${PROJECT_NAME} PUBLIC ZMQ_BUILD_DRAFT_API=1)


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