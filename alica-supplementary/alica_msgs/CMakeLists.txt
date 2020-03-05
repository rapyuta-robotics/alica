cmake_minimum_required(VERSION 2.8.3)
project(alica_msgs)

find_package(catkin REQUIRED message_generation std_msgs geometry_msgs id_ros)

add_message_files(
  DIRECTORY msg FILES
  AllocationAuthorityInfo.msg
  EntryPointRobots.msg
  RoleSwitch.msg
  SolverVar.msg
  SyncReady.msg
  AlicaEngineInfo.msg 
  PlanTreeInfo.msg
  SolverResult.msg
  SyncData.msg
  SyncTalk.msg
  StringTuple.msg
  AgentQuery.msg
  AgentAnnouncement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  id_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alica_msgs
   CATKIN_DEPENDS std_msgs geometry_msgs message_runtime id_ros
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)
