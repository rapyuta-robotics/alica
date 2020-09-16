cmake_minimum_required(VERSION 3.5.1)
project(id_ros)
 
find_package(catkin REQUIRED roscpp message_generation)

add_message_files(
  FILES
  ID.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  # INCLUDE_DIRS
  #  LIBRARIES ${PROJECT_NAME}_generated_messages_cpp
  CATKIN_DEPENDS roscpp message_runtime
)
