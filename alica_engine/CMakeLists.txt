cmake_minimum_required(VERSION 2.8.3)
project(alica_engine)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp system_config fsystem autodiff event_handling)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS program_options)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
 
## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES alica_engine
  CATKIN_DEPENDS roscpp fsystem
#  DEPENDS tinyxml
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include test/include ${catkin_INCLUDE_DIRS})


file(GLOB_RECURSE alica_SOURCES "src/*.cpp")

## Declare a cpp library
 add_library(alica_engine
	${alica_SOURCES};
 )
  add_library(test_behaviour_creator
	test/src/Attack.cpp;
	test/src/AttackOpp.cpp;
	test/src/DefendMid.cpp;
	test/src/MidFieldStandard.cpp;
	test/src/Tackle.cpp;
	test/src/TestBehaviourCreator.cpp;
 )
## Declare a cpp library
 ##add_library(alica
   ##src/engine/AlicaEngine.cpp
   ##src/engine/PlanRepository.cpp
   ##src/engine/parser/PlanParser.cpp
   ##src/engine/parser/ModelFactory.cpp
   ##src/engine/parser/tinyxml2.cpp
   ##src/engine/model/AlicaElement.cpp
   ##src/engine/model/Plan.cpp
   ##src/engine/model/AbstractPlan.cpp
   ##src/engine/model/RuntimeCondition.cpp
   ##src/engine/model/Condition.cpp
   ##src/engine/model/EntryPoint.cpp
   ##src/engine/model/TaskRepository.cpp
   ##src/engine/model/Task.cpp
   ##src/engine/model/Role.cpp
   ##src/engine/model/Characteristic.cpp
   ##src/engine/model/SyncTransition.cpp
   ##src/engine/model/Transition.cpp
   ##src/engine/model/Quantifier.cpp
   ##src/engine/model/Variable.cpp
   ##src/engine/model/Behaviour.cpp
   ##src/engine/model/State.cpp
   ##src/engine/model/PlanType.cpp
   ##src/engine/model/BehaviourConfiguration.cpp
   ##src/engine/model/RoleDefinitionSet.cpp
   ##src/engine/model/Capability.cpp
   ##src/engine/UtilityFunction.cpp
   ##src/engine/model/PreCondition.cpp
   ##src/engine/BasicBehaviour.cpp
   ##src/engine/model/CapValue.cpp;
 ## )
 target_link_libraries(alica_engine ${catkin_LIBRARIES})
 add_dependencies(alica_engine ${catkin_LIBRARIES})
 

## Declare a cpp executable
# add_executable(test_base src/TestBase.cpp)
# 
# target_link_libraries(test_base
#   ${catkin_LIBRARIES} alica
# )
# add_dependencies(test_base ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS alica_engine alica_engine_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
file(GLOB_RECURSE test_alica_SOURCES "test/src/*.cpp")
 catkin_add_gtest(${PROJECT_NAME}-test ${test_alica_SOURCES})
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
