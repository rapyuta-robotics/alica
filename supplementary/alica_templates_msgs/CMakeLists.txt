cmake_minimum_required(VERSION 3.5.1)
project(alica_templates_msgs)

find_package(catkin REQUIRED message_generation actionlib_msgs)

add_action_files(
    DIRECTORY action
    FILES Dummy.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alica_msgs
   CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)
