cmake_minimum_required(VERSION 3.5.1)
project(alica_msgs)

find_package(ament_cmake QUIET)
if (ament_cmake_FOUND)
  find_package(ament_cmake REQUIRED)

  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/AllocationAuthorityInfo.msg
    msg/AllocationAuthorityInfoList.msg
    msg/EntryPointRobots.msg
    msg/RoleSwitch.msg
    msg/SolverVar.msg
    msg/SyncReady.msg
    msg/SyncReadyList.msg
    msg/AlicaEngineInfo.msg
    msg/PlanTreeInfo.msg
    msg/PlanTreeInfoList.msg
    msg/SolverResult.msg
    msg/SolverResultList.msg
    msg/SyncData.msg
    msg/SyncTalk.msg
    msg/SyncTalkList.msg
    msg/StringTuple.msg
    msg/AgentQuery.msg
    msg/AgentQueryList.msg
    msg/AgentAnnouncement.msg
    msg/AgentAnnouncementList.msg
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

else (ament_cmake_FOUND)
  find_package(catkin REQUIRED message_generation std_msgs geometry_msgs)

  add_message_files(
    DIRECTORY msg FILES
    AllocationAuthorityInfo.msg
    AllocationAuthorityInfoList.msg
    EntryPointRobots.msg
    RoleSwitch.msg
    SolverVar.msg
    SyncReady.msg
    SyncReadyList.msg
    AlicaEngineInfo.msg
    PlanTreeInfo.msg
    PlanTreeInfoList.msg
    SolverResult.msg
    SolverResultList.msg
    SyncData.msg
    SyncTalk.msg
    SyncTalkList.msg
    StringTuple.msg
    AgentQuery.msg
    AgentQueryList.msg
    AgentAnnouncement.msg
    AgentAnnouncementList.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES alica_msgs
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
  #  DEPENDS system_lib
  )

  include_directories(
   ${catkin_INCLUDE_DIRS}
  )
endif(ament_cmake_FOUND)
