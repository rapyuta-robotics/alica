cmake_minimum_required(VERSION 3.5.1)
project(alica_ros_turtlesim)

include(../../../cmake_flags/cflags.cmake)
alica_compile_flags()
##################
## Dependencies ##
##################

set(PKG_DEPS
    roscpp
    alica_ros_proxy
    turtlesim
    geometry_msgs
)

find_package(Boost COMPONENTS system filesystem REQUIRED)
if (NOT Boost_FOUND)
      message(FATAL_ERROR "Fatal error: Boost required.\n")
endif (NOT Boost_FOUND)

##Yaml
find_package(yaml-cpp REQUIRED)
find_package(alica_solver_interface)
find_package(alica_engine REQUIRED)
find_package(constraintsolver REQUIRED)
find_package(autodiff REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})
find_package(alica_dynamic_loading REQUIRED)

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
    INCLUDE_DIRS include alica/Expr/include
    DEPENDS autodiff alica_dynamic_loading

)


###########
## Build ##
###########

include_directories(
    include
    alica/Expr/include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# Add the autogenerated cpp files as a library
file(GLOB_RECURSE autogen_SOURCES "alica/Expr/src/*.cpp")

add_executable(${PROJECT_NAME}
    ${autogen_SOURCES}
    src/turtle.cpp
    src/world_model.cpp
    src/base_node.cpp
    src/base.cpp
)

target_link_libraries(${PROJECT_NAME}
                        Boost::system
                        Boost::filesystem
                        ${CMAKE_DL_LIBS}
                        ${catkin_LIBRARIES}
                        ${YAML_CPP_LIBRARIES}
                        alica_dynamic_loading
                        autodiff
                        constraintsolver
)

add_subdirectory(alica_turtlesim_library)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch alica
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)