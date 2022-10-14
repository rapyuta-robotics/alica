cmake_minimum_required(VERSION 3.5.1)
project(alica_engine)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

# Set up utils to allow Colcon/catkin to find the packages
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}/cmake) 


##Yaml
find_package(yaml-cpp REQUIRED)

# Imports
find_package(alica_solver_interface REQUIRED)


###########
## Build ##
###########

include_directories(include)

add_library(${PROJECT_NAME} SHARED 
  src/engine/AlicaClock.cpp
  src/engine/AlicaContext.cpp
  src/engine/AlicaEngine.cpp
  src/engine/Assignment.cpp
  src/engine/BasicBehaviour.cpp
  src/engine/BasicCondition.cpp
  src/engine/BasicPlan.cpp
  src/engine/BasicUtilityFunction.cpp
  src/engine/ConfigChangeListener.cpp
  src/engine/DefaultUtilityFunction.cpp
  src/engine/FileSystem.cpp
  src/engine/IAlicaCommunication.cpp
  src/engine/IRoleAssignment.cpp
  src/engine/Logger.cpp
  src/engine/Logging.cpp
  src/engine/PlanBase.cpp
  src/engine/PlanRepository.cpp
  src/engine/RuleBook.cpp
  src/engine/RunnableObject.cpp
  src/engine/RunningPlan.cpp
  src/engine/RuntimeBehaviourFactory.cpp
  src/engine/RuntimePlanFactory.cpp
  src/engine/SimplePlanTree.cpp
  src/engine/StaticRoleAssignment.cpp
  src/engine/TeamObserver.cpp
  src/engine/UtilityFunction.cpp
  src/engine/allocationauthority/AllocationDifference.cpp
  src/engine/allocationauthority/AuthorityManager.cpp
  src/engine/allocationauthority/CycleManager.cpp
  src/engine/allocationauthority/EntryPointRobotPair.cpp
  src/engine/blackboard/BlackboardUtil.cpp
  src/engine/blackboard/KeyMapping.cpp
  src/engine/collections/RobotEngineData.cpp
  src/engine/collections/RobotProperties.cpp
  src/engine/collections/SuccessCollection.cpp
  src/engine/collections/SuccessMarks.cpp
  src/engine/collections/Variant.cpp
  src/engine/constraintmodul/ConditionStore.cpp
  src/engine/constraintmodul/ProblemDescriptor.cpp
  src/engine/constraintmodul/ProblemPart.cpp
  src/engine/constraintmodul/Query.cpp
  src/engine/constraintmodul/ResultEntry.cpp
  src/engine/constraintmodul/UniqueVarStore.cpp
  src/engine/constraintmodul/VariableSyncModule.cpp
  src/engine/default/DefaultConditions.cpp
  src/engine/default/DefaultTransitionConditionCreator.cpp
  src/engine/expressionhandler/BasicFalseCondition.cpp
  src/engine/expressionhandler/BasicTrueCondition.cpp
  src/engine/expressionhandler/DummyConstraint.cpp
  src/engine/expressionhandler/ExpressionHandler.cpp
  src/engine/logging/AlicaDefaultLogger.cpp
  src/engine/logging/AlicaLogger.cpp
  src/engine/logging/Logging.cpp
  src/engine/model/AbstractPlan.cpp
  src/engine/model/AlicaElement.cpp
  src/engine/model/Behaviour.cpp
  src/engine/model/Condition.cpp
  src/engine/model/ConfAbstractPlanWrapper.cpp
  src/engine/model/Configuration.cpp
  src/engine/model/EntryPoint.cpp
  src/engine/model/FailureState.cpp
  src/engine/model/ForallAgents.cpp
  src/engine/model/Parameter.cpp
  src/engine/model/Plan.cpp
  src/engine/model/PlanType.cpp
  src/engine/model/PostCondition.cpp
  src/engine/model/PreCondition.cpp
  src/engine/model/Quantifier.cpp
  src/engine/model/Role.cpp
  src/engine/model/RoleSet.cpp
  src/engine/model/RuntimeCondition.cpp
  src/engine/model/State.cpp
  src/engine/model/SuccessState.cpp
  src/engine/model/Synchronisation.cpp
  src/engine/model/Task.cpp
  src/engine/model/TaskRepository.cpp
  src/engine/model/TerminalState.cpp
  src/engine/model/TransitionCondition.cpp
  src/engine/model/TransitionConditionRepository.cpp
  src/engine/model/Transition.cpp
  src/engine/model/VariableBinding.cpp
  src/engine/model/Variable.cpp
  src/engine/modelmanagement/ModelManager.cpp
  src/engine/modelmanagement/factories/AbstractPlanFactory.cpp
  src/engine/modelmanagement/factories/BehaviourFactory.cpp
  src/engine/modelmanagement/factories/BlackboardBlueprintFactory.cpp
  src/engine/modelmanagement/factories/ConditionFactory.cpp
  src/engine/modelmanagement/factories/ConfAbstractPlanWrapperFactory.cpp
  src/engine/modelmanagement/factories/ConfigurationFactory.cpp
  src/engine/modelmanagement/factories/EntryPointFactory.cpp
  src/engine/modelmanagement/factories/Factory.cpp
  src/engine/modelmanagement/factories/KeyMappingFactory.cpp
  src/engine/modelmanagement/factories/PlanFactory.cpp
  src/engine/modelmanagement/factories/PlanTypeFactory.cpp
  src/engine/modelmanagement/factories/PostConditionFactory.cpp
  src/engine/modelmanagement/factories/PreConditionFactory.cpp
  src/engine/modelmanagement/factories/QuantifierFactory.cpp
  src/engine/modelmanagement/factories/RoleFactory.cpp
  src/engine/modelmanagement/factories/RoleSetFactory.cpp
  src/engine/modelmanagement/factories/RuntimeConditionFactory.cpp
  src/engine/modelmanagement/factories/StateFactory.cpp
  src/engine/modelmanagement/factories/SynchronisationFactory.cpp
  src/engine/modelmanagement/factories/TaskFactory.cpp
  src/engine/modelmanagement/factories/TaskRepositoryFactory.cpp
  src/engine/modelmanagement/factories/TerminalStateFactory.cpp
  src/engine/modelmanagement/factories/TransitionConditionFactory.cpp
  src/engine/modelmanagement/factories/TransitionConditionRepositoryFactory.cpp
  src/engine/modelmanagement/factories/TransitionFactory.cpp
  src/engine/modelmanagement/factories/VariableBindingFactory.cpp
  src/engine/modelmanagement/factories/VariableFactory.cpp
  src/engine/planselector/IAssignment.cpp
  src/engine/planselector/PartialAssignment.cpp
  src/engine/planselector/PartialAssignmentPool.cpp
  src/engine/planselector/PlanSelector.cpp
  src/engine/planselector/TaskAssignmentProblem.cpp
  src/engine/syncmodule/SynchronisationProcess.cpp
  src/engine/syncmodule/SyncModule.cpp
  src/engine/syncmodule/SyncRow.cpp
  src/engine/teammanager/Agent.cpp
  src/engine/teammanager/TeamManager.cpp
  src/engine/util/cityhash.cpp
  src/engine/util/ConfigPathParser.cpp
)

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
  ${YAML_CPP_LIBRARIES} 
  alica_solver_interface
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${CMAKE_PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/ 
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


# Export targets to make package libraries
export(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Targets.cmake
)

install(EXPORT ${CMAKE_PROJECT_NAME}Targets
  FILE ${CMAKE_PROJECT_NAME}Targets.cmake
  DESTINATION ${ConfigPackageLocation}
)

# Installing files to make a find_package()-able package
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.9.4
  COMPATIBILITY SameMajorVersion
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${CMAKE_PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${ConfigPackageLocation}
  PATH_VARS INCLUDE_INSTALL_DIR
)

install(FILES 
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${ConfigPackageLocation}
)


#############
## Doxygen ##
#############

# The BUILD_DOXYGEN option can be set via
# catkin build <package_name> --cmake-args -DBUILD_DOXYGEN=ON
option(BUILD_DOXYGEN "Generate Doxygen Documentation")

find_package(Doxygen QUIET)
if (DOXYGEN_FOUND AND BUILD_DOXYGEN)
  # set input and output files
  set(DOCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../docs/doxygen)
  set(DOXYGEN_IN ${DOCS_DIR}/../Doxyfile.in)
  set(DOXYGEN_OUT ${DOCS_DIR}/${PROJECT_NAME}/Doxyfile)

  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT})
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target(${PROJECT_NAME}_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen into ${DOCS_DIR}"
    VERBATIM
  )
elseif (BUILD_DOXYGEN)
  message("Doxygen needs to be installed to generate the Doxygen Documentation for ${PROJECT_NAME}")
endif (DOXYGEN_FOUND AND BUILD_DOXYGEN)
