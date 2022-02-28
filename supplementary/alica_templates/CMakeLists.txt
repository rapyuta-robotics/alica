cmake_minimum_required(VERSION 3.5.1)
project(alica_templates)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    fsystem
    alica_engine
    alica_dummy_proxy
    alica_ros_proxy
    alica_simple_solver
    alica_common_config
    alica_solver_interface
    alica_test_utility
    alica_msgs
    constraintsolver
    autodiff
    alica_ros_proxy
    actionlib_msgs
    actionlib
    genmsg
)

# add_action_files(
#     DIRECTORY action
#     FILES Dummy.action
# )

# generate_messages(
#     DEPENDENCIES actionlib_msgs std_msgs
# )

include_directories(include)

catkin_package(
  INCLUDE_DIRS include Expr/include
  LIBRARIES autogen_code_alica_templates
  CATKIN_DEPENDS alica_engine alica_dummy_proxy alica_solver_interface alica_simple_solver constraintsolver alica_ros_proxy actionlib_msgs actionlib
)

include_directories(include Expr/include ${catkin_INCLUDE_DIRS})
file(GLOB_RECURSE autogen_SOURCES "Expr/src/*.cpp")
add_library(autogen_code_alica_templates ${autogen_SOURCES};)
target_link_libraries(autogen_code_alica_templates ${catkin_LIBRARIES})
add_dependencies(autogen_code_alica_templates ${catkin_EXPORTED_TARGETS})
