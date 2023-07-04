^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_test_utility
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Remove eventDriven & deferring of behaviours (`#494 <https://github.com/rapyuta-robotics/alica/issues/494>`_)
  eventDriven is removed because:
  - there is no use case for this functionality so far
  - external code cannot possibly hold a reference to a behaviour because
  there is no such external API & even if such a reference is obtained
  it can be invalidated at anytime because the behaviour can be destroyed
  at any time & therefore triggering the  behaviour in the current way
  will likely lead to crashes
  - the functionality of eventDriven can be replicated by having
  a state machine with 2 states (A & B) & a transition. The transition
  will be true when the trigger condition is true & the code that needs to
  be executed (i.e. the code that was in run() method earlier) will now
  be in the init method of B
  deferring is removed because:
  - there is no use case for this functionality so far
  - it is not currently implemented
  - there is no easy way to implement it either
  - same functionality can be had by having an additional state that
  waits for the deferring period before transitioning to the state with
  the attached behaviour
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
* Remove configuration leftovers (`#485 <https://github.com/rapyuta-robotics/alica/issues/485>`_)
  remove configuration leftovers
* Removing Configuration (`#469 <https://github.com/rapyuta-robotics/alica/issues/469>`_)
  * Removing Configuration
  * removed Configuration strings
  * Removing files
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Add backward compatible test context constructor for config path (`#449 <https://github.com/rapyuta-robotics/alica/issues/449>`_)
* Add ability to find config files in multiple dirs (`#429 <https://github.com/rapyuta-robotics/alica/issues/429>`_)
  * Add ability to find config files in multiple dirs
  - The engine now has a set of directories where it looks for config
  files recursively
  - If the roleset with the given file name is not found (recursive
  search), then the engine looks for any roleset file in the specified
  directories but not recursively
  - The beh & cnd files are now referred to only by their file name &
  not by their relative path to the plans folder
  - The plan, role & task config keys are no longer required since the
  search is recursive & hence these are removed
  * Accept multiple config folders in the AlicaContext
  * Consider only the file name in referenced elements
  If the referenced element has a path instead of a name only consider the
  name Eg. if a behaviour is referenced by a path (relative or absolute)
  to the .beh file then only consider the file name since we perform
  a search for the file now instead of directly using the path.
  * Fix tests
  * Fix turtlesim
  * Review comments: Change unordered_set to vector
  For config paths container, use vector instead of unordered_set so that
  user order can be maintained. In this case, the search for files will
  proceed from index 0 which means that will also be the order of priority
  to find files.
  Couple of other very minor fixes
  * Add multi config folder plan parsing unit test
  * Fix comment
  * Remove unnecessary include
  * Minor fix
  * More fixes
  * Log fixes
  * Fix review comments
  * Check if logging is initialised in the logger
  * Fix plan parser test
  * Bring back single config path constructor as deprecated
  * Update alica_engine/src/engine/modelmanagement/ModelManager.cpp
  Co-authored-by: Maksim Derbasov <ntfs.hard@gmail.com>
  * Review comment
  ---------
  Co-authored-by: Maksim Derbasov <ntfs.hard@gmail.com>
* Revert "removing Configuration" (`#434 <https://github.com/rapyuta-robotics/alica/issues/434>`_)
  Revert "removing Configuration (`#428 <https://github.com/rapyuta-robotics/alica/issues/428>`_)"
  This reverts commit 31bc444bb52f30aaba4407fa9d593c77ce88383a.
* removing Configuration (`#428 <https://github.com/rapyuta-robotics/alica/issues/428>`_)
  * removing Configuration
  * Clean-up
  * Review
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Use blackboard to store WorldModels for libraries  (`#360 <https://github.com/rapyuta-robotics/alica/issues/360>`_)
  * step 001
  * fix compilation for alica_tests
  * step 003
  * test compilation ok execution still fails
  * some tests works
  * alica_tests fixed
  * fix supplementary_test
  * fix turtlesim demo
  * some cleanup
  * minor
  * fix format
  * add test, minor cleanup
  * minor
  * fix format
  * cleanup step 001
  * fix format
  * fix pedantic
  * removed string namespace step 001
  * fix
  * fix code gen for alica_tests
  * fix code gen for supplementary_tests
  * fix code gen for turtlesim
  * minor
  * fix alica_tests
  * fix turtlesim
  * fix const_cast
  * fix format
  * some fixes
  * remove worldModel name
  * remove worldModel name step 002
  * remove worldModel name step 003
  * fix PR comments step 001
  * fix PR comments step 002
  * fix PR comments step 003
  * fix PR comments step 005
  * fix PR comments step 005
  * minor
  * test fix
  * fix attempt for TestTracing.h
  * fix pointers
  * fix format
  * fix minor
  * fix trace test
  * Update alica_engine/include/engine/AlicaContext.h
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  * Update alica_engine/include/engine/AlicaContext.h
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  * fix comments step001
  * fix comments step002
  * fix comments step003
  * fix format
  * minor fix
  * fix minor
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Improve tests (`#365 <https://github.com/rapyuta-robotics/alica/issues/365>`_)
  * WIP
  * Minor fixes
  * Add sample test plans
  * Modify transitions, gen code
  * WIP
  * way to set transitions
  * First run without crash
  * First working test
  * Fix format
  * Plan success test
  * Minor change
  * WIP
  * Regenerate code
  * Add isStateActive method
  * Multi plan instance test & fixes
  * Fix relative diretory of behaviour:
  * Reenable old tests
  * Add failure reason
  * Comments & print failure info
  * Address review comments
  - Add STEP_UNTIL_ASSERT_TRUE macro
  - Add comments for the tests
  - Add missing comment for the return value of setTransitionCond
  * Review comment - Add step until assert eq
  * Fixes
  - Handle lvalues in set API correctly i.e. use std::decay before
  determining the type (fixes compile error)
  - Add tests for the above
  - Specify types & default values for blackboard keys now that they are
  supported
* Add support for legacy transition conditions in alica (`#352 <https://github.com/rapyuta-robotics/alica/issues/352>`_)
  * generate LegacyTransitionConditionCreator, generate evaluate functions in plan files
  * add support for legacy conditions to engine
  * update codegen for legacy conditions
  * format
  * update codegen jar
  * support non updated pml files, dont create TransitionConditionCreator if no condition file exists
  * fix some cmake issues
  * fix CMakeLists of alica_dummy_tracing
  * initialize transition condition objects for legacy transition conditions
  * use int64_t for ids
  * add missing setTracing implementation to RunnableObject
  * remove unused include, fix typo in comment
  * remove transitionConditon creation when no conditionRepository.cnd file is present, remove distinction between using new and legacy transitions, add preConditionId to TransitionContext, use preConditionId in legacyTransitionConditionCreator
  * format
  * add deprecated comment to preConditionId
  * remove alica_common_config from cmake files
  * add conditionId to legacyTransitionCondition createConditions
  * Add test for legacy conditions, regenerate files (`#354 <https://github.com/rapyuta-robotics/alica/issues/354>`_)
  * add test for legacy conditions, regenerate files
  * regenerate legacyTransitionConditionCreator
  * regenerate
  * regenerate code, add conditionId to legacyConditionCreator::createCondition args
  * remove msg from run
  * pass preConditionId to transitionConditionContext in dynamicloadercreator test, use precondition in LegacyConditionCreator in alica_tests
  * format
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Removed msg param  from  BasicBehaviour::run  (`#358 <https://github.com/rapyuta-robotics/alica/issues/358>`_)
  * removed msg to run in behaviour
  * fix code generation
  * jar
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Fix test step until (`#349 <https://github.com/rapyuta-robotics/alica/issues/349>`_)
  * first step
  * add STEP_UNTIL_VECT and fix tests, move CounterClass
  * fix format
  * Update Util.h
  * fix format
  * fix STEP_UNTIL conditions
  * Update alica_test_utility/include/alica/test/Util.h
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
  * fix
  * fix name
  * fix
  * fix
  * missing fix
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Run precommit on all files (`#340 <https://github.com/rapyuta-robotics/alica/issues/340>`_)
* Removed Redundant cmake Libraries (`#336 <https://github.com/rapyuta-robotics/alica/issues/336>`_)
* Move Non-ROS packages to CMake (`#325 <https://github.com/rapyuta-robotics/alica/issues/325>`_)
  * Changed alica_common_config to a CMake package and updated includes
  * Ported alica_solver_interface to plain CMake package
  * Ported autodiff to Cmake package
  * Ported the alica_engine package to pure cmake
  * Ported alica_dummy_proxy to a CMake package
  * Ported alica_simple_solver and alica_dummy_tracing to CMake packages
  * Ported alica_test_utility to a Cmake package
  * Cleaned up files and added comments
  * Changed alica_common_config to a plain CMake Package
  * Ported alica_solver_interface to plain CMake package
  * Ported autodiff to Cmake package
  * Ported the alica_engine package to pure cmake
  * Ported alica_dummy_proxy to a CMake package
  * Ported alica_simple_solver and alica_dummy_tracing to CMake packages
  * Ported alica_test_utility to a Cmake package
  * Documented changes to CMakeLists.txt and removed unused code
  * Removed extra 'include directories' as per PR comments
  * Updated version numbers to 0.9.4
  * Removed GLOB_RECURSE from cmake except for autogen code
  * Removed duplicate target_sources
* PR10 - Remove AlicaEngine::this from RuntimeBehaviourFactory  (`#297 <https://github.com/rapyuta-robotics/alica/issues/297>`_)
  * still tests not working
  * fix some crashes, some tests fails
  * removed nont working test
  * fix formatting
  * check added
  * fix some tests
  * fix veerajsk comments step001
  * fix veerajsk comments step002
  * fix veerajsk comments step003
  * fix test_config_change test
  * fix test_failure_handling test
  * fix test_task_assignment test
  * fix test_alica_authority
  * fix comments
  * new move constructor
  * fix problem
  * PR4
  * PR5
  * PR6
  * remove AlicaEngin::this from teammanager
  * fix formatting
  * fix comments on review
  * Remove AlicaEngine::this from Logger
  * rebase
  * first step
  * step 002
  * step 004
  * solvers fix
  * fix code for test
  * fix test
  * try fix test test_alica_condition_plan.cpp
  * minor comment
  * conflict resolved
  * fix formatting
  * format
  * step 002
  * step 004
  * fix code for test
  * try fix test test_alica_condition_plan.cpp
  * step 001
  * fix test test_alica_scheduling
  * rebase
  * fix formatting
  * rebase
  * PR7 - Remove AlicaEngine::this from teammanager (`#290 <https://github.com/rapyuta-robotics/alica/issues/290>`_)
  * still tests not working
  * fix some crashes, some tests fails
  * tests fix step 001
  * removed nont working test
  * fix formatting
  * check added
  * fix deprecated
  * fix some tests
  * fix veerajsk comments step001
  * fix veerajsk comments step002
  * fix veerajsk comments step003
  * test fix
  * fix test_config_change test
  * fix test_failure_handling test
  * fix test_task_assignment test
  * fix test_alica_authority
  * fix supplementary test
  * fix formatting
  * fix comments
  * fix comments step002
  * new move constructor
  * fix problem
  * first step
  * PR4
  * PR5
  * PR6
  * remove AlicaEngin::this from teammanager
  * fix formatting
  * fix comments on review
  * fix as for PR comments
  * revert jar files
  * revert
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
  * PR8 - Remove AlicaEngine::this from Logger (`#293 <https://github.com/rapyuta-robotics/alica/issues/293>`_)
  * still tests not working
  * fix some crashes, some tests fails
  * tests fix step 001
  * removed nont working test
  * fix formatting
  * check added
  * fix deprecated
  * fix some tests
  * fix veerajsk comments step001
  * fix veerajsk comments step002
  * fix veerajsk comments step003
  * test fix
  * fix test_config_change test
  * fix test_failure_handling test
  * fix test_task_assignment test
  * fix test_alica_authority
  * fix supplementary test
  * fix formatting
  * fix comments
  * fix comments step002
  * new move constructor
  * fix problem
  * first step
  * PR4
  * PR5
  * PR6
  * remove AlicaEngin::this from teammanager
  * fix formatting
  * fix comments on review
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
  * Small Fix in TestContext (`#309 <https://github.com/rapyuta-robotics/alica/issues/309>`_)
  * Update TestContext.cpp
  * review comment
  * delaystart = ture
  * Update conditions (`#307 <https://github.com/rapyuta-robotics/alica/issues/307>`_)
  * update alica_tests
  * regenerate files
  * update supplementary tests
  * update turtlesim
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  * fix
  * update plandesigner readme (`#308 <https://github.com/rapyuta-robotics/alica/issues/308>`_)
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  * fix rebase
  * fix rebase
  * fix merge
  * format fix
  * fix comment on PR and remove include and fw declaration unused
  * fix
  * fix
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
  Co-authored-by: Prajapati-Pawan <100663745+Prajapati-Pawan@users.noreply.github.com>
  Co-authored-by: bjoernschroeder <39679198+bjoernschroeder@users.noreply.github.com>
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Small Fix in TestContext (`#309 <https://github.com/rapyuta-robotics/alica/issues/309>`_)
  * Update TestContext.cpp
  * review comment
  * delaystart = ture
* Condition Factory `#1 <https://github.com/rapyuta-robotics/alica/issues/1>`_ (`#261 <https://github.com/rapyuta-robotics/alica/issues/261>`_)
  * dummy files
  * generate condition functions & new creator
  * add ITransitionPreConditionCreator and TransitionPreConditionFactory
  * add TransitionPreConditionCreator to AlicaCreators
  * initialize and provide access to TransitionConditionFactory
  * run new transition conditions, store inputs of transition conditoins
  * implement and create TransitionCondition model object
  * - implement runtime transition condition objects
  - store runtime transition condition objects in basicPlans
  - create runtime transition objects when the runningPlan is activated
  - refactoring
  * - fix review comments
  * - pass WorldModel to transitionCondition evaluation
  - move setInput/Output to KeyMapping
  - movegetParentWrapperId to RunningPlan
  * update codegen
  * - Read and store keyMapping of transitions in transition model object
  - add DefaultTransitionConditionCreator
  - fix getParentWrapperId
  * fix review comments
  * add name to conditionRepository node
  * move default implementation into default folder
  * - fix includes
  - fix creatorCallback names
  * remove todo for constraints
  * update codegeneration
  * use correct TransitionConditions for generating callbacks, remove transition generation from plans
  * remove tmp node from modelmanager
  * update TestContext to use TransitionConditionCreator
  * remove old code, add protected region to conditions header, update codegen jar
  * - remove BasicTransitionCondition (`#294 <https://github.com/rapyuta-robotics/alica/issues/294>`_)
  - move blackboard and callback to TransitionCondition model object
  - replace RuntimeTransitionConditionFactory with TransitionConditionCallbackFactory
  - set evalCallback for TransitionConditions in engine::init()
  - remove BasicTransitionCondition from BasicPlans
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  * - remove TransitionConditionCallbackFactory
  - create TransitionCondition in initTransitionConditions in engine
  - remove TransitionConditionContext struct
  - assert that TransitionCondition callback exists before calling
  * remove forward declaration
  * move setInput / setOutput to BlackboardUtil
  * add iostream and debug_output includes
  * Condition Factory `#3 <https://github.com/rapyuta-robotics/alica/issues/3>`_ Update turtlesim for new conditions (`#286 <https://github.com/rapyuta-robotics/alica/issues/286>`_)
  * - regenerate code
  - move implementation of transitions to conditions.cpp
  - update base
  * regenerate files
  * fix turtlesim after merge, move addSolver call after initialization of context
  * Condition Factory `#4 <https://github.com/rapyuta-robotics/alica/issues/4>`_ Update alica_tests to use new conditions (`#291 <https://github.com/rapyuta-robotics/alica/issues/291>`_)
  * update tests to use new conditions
  * update TestContext
  * regenerate
  * Condition Factory `#5 <https://github.com/rapyuta-robotics/alica/issues/5>`_ Update supplementary tests for new transition conditions (`#292 <https://github.com/rapyuta-robotics/alica/issues/292>`_)
  * use new TransitionConditons
  * regenerate code, move generated files from autogenerated to Expr
  * format
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  * fix review comments
  * format
  Co-authored-by: Athish T <athish.thirumal@rapyuta-robotics.com>
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* Removing AlicaEngine::this from TeamObserver (`#282 <https://github.com/rapyuta-robotics/alica/issues/282>`_)
  Removing AlicaEngine references from TeamObserver and subclasses. Delay initialization of the engine in tests.
* Add configurable auto failure handling capability (`#279 <https://github.com/rapyuta-robotics/alica/issues/279>`_)
  * Add configurable auto failure handling capability
  Add a AutoFailureHandling config that can be used to enable/disable
  auto failure handling for a plan in the engine
  * Add config to all alica.yaml files
  * Enable plan abort & make tests more robust
  The plan abort rule should not be considered an auto failure handling
  behaviour so that the failure can be immediately handled by the higher
  levels.
  Some of the tests could pass if a plan restarted when they should fail,
  this is suitably addressed by ensuring transitions are unset when they
  are no longer needed to be enabled & by ensuring the plan init is only
  executed once
* Creators fixes (`#264 <https://github.com/rapyuta-robotics/alica/issues/264>`_)
  * Creators fixes
  * Ensure stepEngine does not deadlock
  * Various fixes with creators
  * More fix
  * Revert mistaken change in tag
* Get rid of BehaviourPool & PlanPool (`#247 <https://github.com/rapyuta-robotics/alica/issues/247>`_)
  * Get rid of BehaviourPool
  * More cleanup
  * Further cleanup
  * Simplify construction of behaviour, new argument shouldm't require app code regeneration
  * Introduce PlanContext
  * Update alica tests
  * Update supplementary tests
  * Update alica test utility
  * Enable git lfs for jar files
  * Update readme
  * Working turtlesim - stress tested
  * Deprecate older init api
  * Fix tests
  * Fix tests
  * Final test fixes
  * Format
  * cleanup
  * Review comments fixes
  * Remove PlanPool (`#259 <https://github.com/rapyuta-robotics/alica/issues/259>`_)
  * Remove PlanPool
  * Fix reactivation of plan
  - Ignore duplicate calls to start & stop in RunnableObject
  - Stop the basicPlan on reactivation & allocate a new BasicPlan object
  if the plan has changed
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
  * Address review comment
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
* Pass context in behaviour and plan creation and add codegeneration jar files (`#249 <https://github.com/rapyuta-robotics/alica/issues/249>`_)
  * Simplify construction of behaviour, new argument shouldn't require app code regeneration
  * Introduce PlanContext
  * Update alica tests
  * Update supplementary tests
  * Update alica test utility
  * Enable git lfs for jar files
  * Update readme
* Evaluate transitions after plan init is executed (`#229 <https://github.com/rapyuta-robotics/alica/issues/229>`_)
  * Evaluate transitions after plan init is executed
  This ensures that the variables used in the transitions are initialized
  by the onInit for that plan before they are used
  * Account for basic plan being null before plan is started
  * Make flag atomic & call after init is executed
  * Fix for plan type in plan pool
  * Fixes for reliable tests (`#230 <https://github.com/rapyuta-robotics/alica/issues/230>`_)
  * Fix compile error
  * Check if in context in returning if init is executed
  * Fixes
  * Test fix
  * more reliable scheduling test
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Tests for passing parameters (`#214 <https://github.com/rapyuta-robotics/alica/issues/214>`_)
  * create plan for parameter tests, implement setParameters
  * add getBasicPlan utility
  * store parameters in wm
  * implement unittests
  * add remaining blackboard unittests
  * remove unnecessary methods
  * format
  * remove unused variable
  * uncomment tests
  * regenerate
  * change default value for gdb to false
  * remove old code
* Format line to eof (`#222 <https://github.com/rapyuta-robotics/alica/issues/222>`_)
  * Add line at eof in format
  * Dont auto add line
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* Adjacent Plan Succeed Test (`#179 <https://github.com/rapyuta-robotics/alica/issues/179>`_)
  * export plans for test
  * regenerate plans
  * add transitions to worldmodel
  * add unittests
  * fix adjacent test
  * regenerate files
  * improve test, remove singleton wm usage
  * update unittest
  * remove unused variable
  * use structured binding, return amISuccessful of matching node
  * format
  * remove old code
  * remove unused variables
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Format and CI (`#209 <https://github.com/rapyuta-robotics/alica/issues/209>`_)
  * Travis test
  * sudo
  * fix
  * Fix ci source path
  * Run format
  * Clang format 10
  * Reformat
  * Fix precommit clang version
  * focal dist
* Replace singleton worldmodel (`#181 <https://github.com/rapyuta-robotics/alica/issues/181>`_)
  * create worldModel interface, update TestWorldModel to use interface
  * use wm in context for scheduling tests
  * fix schedWM usage
  * fix remaining tests
  * pass worldmodel to behavoíours and plans on construction, pass worldmodel to utilityfunctions and conditions
  * pass worldmodel to summands
  * update generated files and tests
  * update test_utility
  * add worldmodel to RunnableObject
  * regenerate files
  * fix test_utility
  * regenerate turtlesim files
  * regenerate supplementary_tests files, add worldmodel
  * add dummy world model
  * pass worldmodel via constructor / inits, use worldmodel ptr
  * pass worldmodel to cacheEvalData
  * regenerate turtlesim
  * regenerate supplementary_tests
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* fixed test context constructor to use default agentId (`#190 <https://github.com/rapyuta-robotics/alica/issues/190>`_)
* removed usage of identifiers (`#182 <https://github.com/rapyuta-robotics/alica/issues/182>`_)
  * removed usage of identifiers
  * alica-tests passed
  * used AgentId
  * removed id_ros dependency
  * removed id_ros dependencies
  * placed AgentId at types.h, used 0 as default agentId, fixed test, fixed hash combine, etc.
  * replaced notAValidID with InvalidAgentID
  * alica-essentials PR changes
  * alica-supplementary PR changes
  * minor fix
  * ci fixes
  * removed boost::hash_combine
* Merge alica repos (`#183 <https://github.com/rapyuta-robotics/alica/issues/183>`_)
  * remove unneeded dependency
  * changes for correct ros clock API
  * Adapted to new capnzero Sub api
  * removing not required virtuals
  * removed engine stuff
  * remove override
  * Update AlicaROSClock.h
  * Removed unnecessary ";"
  * Moved header to cpp
  * - removed pm_control
  * - adapt to new capnzero api
  * impletemtation of the PersistID option
  * correct small oversight
  * Change return type of AgentID conversion function to unint64_t
  * - removed output
  * - fix init of wrong subscriber
  * - removed robot control
  * Add install targets for catkin install build
  * Add install targets for catkin install build
  * - improved getSelfPath (now it really returns just the path, excluding the executable)
  * Update constructor
  * Update constructor
  * Rectify header folder structure
  1. Move headers in fsystem, system_config & system_util packages
  under <package>/essentials/ directory.
  2. Modify cmake files for above packages to additionally install these
  moved headers to the global include/ directory in addition to
  /include/essentials/ directory. This is to avoid breaking existing
  code and should be removed once the code is modified to include the
  headers using essentials/<header>. Also export these headers to avoid
  breaking existing code & build.
  * Rectify header folder structure
  Move header files (not sub-directories) in constraintsolver/include/
  under constraintsolver/include/constraintsolver/. However, to avoid
  breaking existing code, export these headers & install them
  under global include/ in addition to include/constraintsolver/.
  * Minor cmake fixes
  * - changed id to IdentifierConstPtr
  * Install alica & launch folder in single statement
  * - you can ask a worker if it is running
  * - improved wildcard handling
  * - refactored alica_capnzero_proxy so that msg conversion is available for everyone
  * - removed processmanager, because capnzero version is now available
  * - improved << operator of ids
  * - added popLast for InfoBuffer (useful for using InfoBuffer as queue)
  * - allowed configs to store everything that std::to_string is working for
  * - toString additionally to operator<<
  * - fixed missing dereferenceing in case of << operator
  * - removed legacy stuff
  - added comments
  * - removed unused packages
  - first manual tweaking iteration of the merge results
  * - removed merge artifact
  * - removed unused type
  * - fixed warning
  * - clean up of cmake lists
  * - fix for essentials include
  * -fix for essentials include
  * update cmake and c++14
  * update cmake and c++14
  * Revert "New JSON Plan Format"
  * - manual pr
  * - fix typo
  - fix unit test
  * - fix unit test for id_manager
  * - added some checks with regard to file separators
  * - conversion tool (first commit)
  * - removed build results
  - introduced factory stubs
  * - VariableFactory done
  - AbstractPlanFactory done
  - EntryPointFactory started
  * - ignore test-stuff folder
  * wip
  * - fixed paths given via cli
  - further dev of factory classes
  * - further factories finished (complete plan.pml can be converted now)
  - missing factories: TaskRepo, Behaviour, PlanType, RoleSet
  * - added TaskRepository and Task Factory
  * - finished all factories
  - parameters of behaviour configurations in behaviour factory are wip
  - next step: resolving references for correct serialisation
  * - finished attaching references
  - next step: high lvl control of converting rolesets
  * - refactored the conversion and introduced the Conversion Process class for a better reusable conversion task
  - unified some methods to handle all kind of files...
  - wip: rolesets make it complicated to find the right task repository, but it should work soon...
  * fix issues with rolesets, roles, taskrepositories
  * - fix for malformed or empty names of files
  * - added autogeneration step (wip)
  * - fixed behaviour -> configuration translation
  - added extra checks during conversion
  * - refactored reference management with own collection, in order to allow multiple entries with the same key, if necessary and throw exceptions otherwise
  * - fixes for the conversion (regarding resolving behaviour configuration references)
  - made format_cpp.sh run with arbitrary version of clang-format
  - added the feature to convert all .pml files at once (does not work for rolesets)
  * - changed name of method in context
  * - add event driven field for behaviours
  * - renamed robotId to agentID
  * - removed TYPE macro, because it wasn't used anyway and is actually defined in Identifier.h of IDManager package
  * - regeneration of test files and fixes for new json-format
  - fixed include directory of constraintsolver
  - improved plan-conversion-tool with regard to variable binding in case of plan types
  * - added missing constraint
  * - made turtles example run with new json-format
  * - added new autogenerated files
  - added Readme for the Plan Conversion Tool
  * - added short version of Readme.md
  - reduced the number of necessary parameters in case of standard sub folders for plans, tasks, roles
  * - minor typo
  * - fix copy past failure
  * - fix travis scripts
  * - add parameters for script
  * - added alica dependency libyaml-cpp-dev
  * - add qt5 dependency for alica viewer
  * - added constructor for uint64_T
  * - removed this
  * added the conversion of behaviour configuration parameters
  * Cmake install fixes
  * Cmake install fixes
  * - add missing method (got lost in merge)
  * Add dependency on alica_msgs so they get generated ahead
  * - added two tests for conversion to uint64_t
  - throw proper exception for conversion of ids that are longer than uint64_t
  * - added comment about exception thrown
  - made string const
  * initial changes
  * removed irrelevant line
  * delegating constructor for default
  * Update Readme.md
  * Update Readme.md
  * Update Readme.md
  * Update Readme.md
  * Update Readme.md
  * Update Readme.md
  * - updated image for current command line interface
  * - temp changes for adapting conversion tool
  * Update Readme.md
  * make conversion tool work with general configurations
  * - make git ignore intellij's project files (*.iml)
  * made getWildcardID const
  * adopted tests to use AlicaTestSupportUtility
  * - converted plans of supplementary test to most current format (general configurations)
  * - made trigger little bit more thread safe
  * - adapted supplementary tests to new alica_test_utility
  * - fix ci
  * - fix ci
  * - add build-essential for ci
  * - added build-essential for ci
  * - upgrade system in ci
  * try to fix cmake version in ci
  * - changed travis from xenial to bionic
  * changed ros distro from kinetic to melodic
  * - changed order of commands for ci
  * fix warnings
  * - removed unnecessary condition variable, now everywhere cvVec_mtx is used
  * - fix timer
  * - improved code quality of event_handling
  * - guarded every write-access to boolean control variables with a lock_guard
  * fix ci
  * addressed PR comments:
  - use unique_ptr
  - fused NotifyTimer and Timer
  * made supplementary_tests run without engine getter
  * fix ci
  * fix comparison between signed and unsigned integer
  * - add build essentials to travis script
  * - format
  * - use not-deprecated method
  * fix initialisation and better memory barries for reading started flag
  * - removed Worker class
  * removed EventTrigger
  * - removed comments
  - fixed naming convention
  * - removed behaviour params
  * removed this
  * - adapted to alica::test::Util class for separating TestContext from alica internal tests
  * Introduce default constructor for Identifier
  * Fix operators to not compare invalid Identifiers
  * - minor changes
  - plan layout
  * - rewrite of steps 1-4
  * - update parts of the turtlesim tutorial
  * - updated plan creation step
  * - updated tutorial
  * replace last image
  * - removed test context include
  * missed one intance
  * update config files to new YAML configs
  * remove static function calls, use new AlicaContext constructor
  * use AlicaContextParams for AlicaContext initialization
  * remove setLocalAgentName
  * use only one config file, remove ID
  * - add temporary ignore for alica_viewer to make a release
  * fix supplementary test
  * - remove System Config dependencies from solvers
  - some cleanup
  * - changed conf for constraintsolver tests
  * Alica.yaml per robot
  * Removed Logging (`#48 <https://github.com/rapyuta-robotics/alica/issues/48>`_)
  * removed logging completely
  * Update README.md
  * Use ROS ENV Variable: ROS_DISTRO
  * Update README.md
  * - add cmake macros for install mode
  * Update README.md
  added instruction to source workspace too
  * - removed cnc_geometry, system_config, udp_proxy_generator
  * removed world_model package
  * - improved description and added missing steps
  * fix quantifiers
  * - integrated suggestions from Dmitrii
  * try to fix python enum34 issue
  * removed lines that were commented for testing CI
  * fix: avoid spawning 30 times a second
  * Bas beh sched engine (`#43 <https://github.com/rapyuta-robotics/alica/issues/43>`_)
  Plan init/run/terminate + plan/behaviour scheduling + web based designer
  - Layout the plans using the new web based plan designer
  - Regenerate the code using the new code generation which generates init, run & terminate functionality for plans (analogous to their counterparts in behaviours)
  - Alica ros timer implementation that is used by the engine to execute the run methods of the plans & behaviours at the desired frequency
  Co-authored-by: Karasuma1412 <bjoerninorek@gmail.com>
  Co-authored-by: Karasuma1412 <bjoern.schroder@rapyuta-robotics.com>
  Co-authored-by: bjoernschroeder <bschroederprogramming@gmail.com>
  * Add the alica designer runtime (`#45 <https://github.com/rapyuta-robotics/alica/issues/45>`_)
  * Add the alica designer runtime
  Add the necessary jar files & docker-compose file required to run the
  web based plan designer & the code generation
  * Address review comment
  * update readme for alica designer runtime
  * add synchronization, fix typos
  * update plan designer compose (`#46 <https://github.com/rapyuta-robotics/alica/issues/46>`_)
  * With live debug support
  * Latest designer runs on port 3030 (`#53 <https://github.com/rapyuta-robotics/alica/issues/53>`_)
  * update compose
  * update readme
  * minor changes
  * update ros_turtle_sim for new plan designer (`#47 <https://github.com/rapyuta-robotics/alica/issues/47>`_)
  * fix numbers in readme
  * native mode default to false
  * Release v0.9.2 (`#55 <https://github.com/rapyuta-robotics/alica/issues/55>`_)
  * implement tracing
  * fix inheritance, implement constructor, use string for context
  * add ros, finish MasterPlan trace
  * remove old code
  * change license to MIT
  * split header and cpp files, remove templates, remove setDefaultTags, pass defaultTags via constructor
  * use rawTraceValue
  * update compose
  * update readme
  * minor changes
  * Fix tracing
  - Store the context in the trace because the master trace is deleted
  immediately
  - Delete the master trace after setting the default tags on it
  * Add license to designer
  * Minor fixes
  - No need to store the span context since it is guranteed to be valid
  even after Finish() is called on the span
  - Take the default tags by value & initialize them
  * fix ci - build jaegertracing
  * Take trace collector from env variable
  * add readme
  Co-authored-by: bjoernschroeder <bschroederprogramming@gmail.com>
  Co-authored-by: Athish <athish.thirumal@rapyuta-robotics.com>
  * remove dependencies on supplementary and essentials repo
  * install tracing from supplementary in subdir
  * fix install.sh path
  * fix install.sh path
  * remove nonstd, system_util and alica_viewer, remove clang, gitignore and travis files in subdirs
  * remove Trigger and NotifyTimer usage, replace VarSyncModule timer with AlicaTimer
  * Fix duplicate repo level files
  * simplify folder name and update readme
  * Fix dependency
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
  Co-authored-by: Gautham Manoharan <gautham.manoharan@rapyuta-robotics.com>
  Co-authored-by: gajen <gajendranagar02@gmail.com>
  Co-authored-by: StefanSchmelz <sschmelz64@gmail.com>
  Co-authored-by: Stephan Opfer <opfer@vs.uni-kassel.de>
  Co-authored-by: william <william.bobillet@rapyuta-robotics.com>
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
  Co-authored-by: Stephan Opfer <stephan.opfer@rapyuta-robotics.com>
  Co-authored-by: Stephan Opfer <StephanOpfer@users.noreply.github.com>
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
  Co-authored-by: corot <jsantossimon@gmail.com>
  Co-authored-by: cyberdrk <cyber.drk@gmail.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  Co-authored-by: athish-t <45649503+athish-t@users.noreply.github.com>
  Co-authored-by: Athish <athish.thirumal@rapyuta-robotics.com>
* Merge branch 'rr-devel' into doxygen_integration
* Merge branch 'rr-devel' into gh-pages
* Merge branch 'rr-devel' into fix_parsing_quantifiers
* Merge branch 'rr-devel' into improve_comment_on_id
* Merge pull request `#130 <https://github.com/rapyuta-robotics/alica/issues/130>`_ from rapyuta-robotics/fix_install_mode
  - add cmake macros for install
* - add cmake macros for install
* Bas yaml cfg (`#125 <https://github.com/rapyuta-robotics/alica/issues/125>`_)
  Add support for Yaml based config
  * get rootPath from ros param server
  * update test to use new ModelManager constructor
  * add getter for AlicaEngine
  * remove old constructor, use YAML config to get basePath
  * remove SystemConfig from ModelManagement
  * move reading config values into reloadConfig
  * load config values in reloadConfig, use YAML config in reloadConfig
  * add local agent configs to yaml
  * yaml config node from context in readSelfFromConfig
  * use yaml config to request persistent id in TeamManager
  * use localAgentName to decide which local config data to load
  * use localAgentName alrady stored in variable
  * use yaml config to request agent ID, fix agent names in yaml config
  * use yaml config to load myRole
  * use yaml config to retrieve local agent data
  * remove old code
  * remove all getInstance calls to SystemConfig
  * remove SystemConfig include
  * change getContext to const
  * replace SystemConfig with YAML config
  * move loading config values to reloadConfig
  * replace SystemConfig with YAML config in PlanBase
  * store AlicaEngine ptr in RuleBook
  * Replace SystemConfig with YAML config in RuleBook
  * Replace SystemConfig with YAML config in CycleManager
  * Replace SystemConfig with YAML config in VariableSyncModule
  * store logPath in AlicaContext
  * Replace SystemConfig with YAML config in AlicaEngine
  * Add PathParser
  * Add setOption to AlicaContext
  * Use setOption to set config values
  * remove old code
  * remove old code
  * add setOptions for vector of key-value pairs
  * Replace SystemConfig with YAML config in RunningPlan
  * Remove SystemConfig include
  * add constructors with AlicaEngine ptr as parameter
  * use YAML config to retrieve config values in AbstractPlan
  * Update derived classes of AbstractPlan and factories to pass AlicaEngine ptr to AbstractPlan constructor
  * Move logging from system_util to alica_engine
  * change namespace from essentials to alica on function calls
  * add AlicaEngine ptr as function argument
  * get LogPath from AlicaContext
  * split PathParser into header and implementation
  * add AlicaEngine include
  * move Logging.cpp into right folder
  * remove logPath
  * remove rootPath from AlicaContext
  * remove setConfigPath from AlicaContext
  * remove old code
  * add localAgentName member to AlicaContext
  * move initialization of Objects in AlicaContext into separate buildObjects function
  * change localAgentName in AlicaContext to non static
  * remove SystemConfig shutdown on termination
  * clean includes
  * remove SystemConfig as a dependency of alica_engine package
  * remove system_config as a package dependency of alica_dummy_proxy
  * remove SystemConfig from test_assignment
  * remove SystemConfig include from RobotProperties
  * remove SystemConfig includes, add mutex include
  * remove SystemConfig includes
  * fix spacing
  * remove SystemConfig package from alica_tests
  * remove SystemConfig includes
  * change integer values to unsigned
  * add ConfigChangeListener
  * implement ConfigChangeListener
  * implement configChangeListener interface in AlicaEngine
  * remove AlicaOptions
  * implement ConfigChangeListener interface in RuleBook
  * implement ConfigChangeListener interface in PlanBase
  * implement ConfigChangeListener in CycleManager
  * add override token
  * implement ConfigChangeListener in VariableSyncModule
  * implement ConfigChangeListener in AbstractPlan
  * implement ConfigChangeListener in ModelManager
  * notify listeners of config changes
  * add subscribe / unsubscribe member functions to AlicaContext
  * subscribe to config changes
  * Block setting config values after initialization
  * remove redundant rootPath param request
  * remove comment
  * simplify setOptions
  * update for loop of unsubscribe
  * update yaml test
  * add config change test
  * update agent values on config change
  * add updateAgentValues member function
  * change visibility of initConfig to private
  * get engine by using AlicaTestsEngineGetter
  * use type instead of auto for AlicaEngine
  * remove getEngine
  * rename PathParser to ConfigPathParser
  * use std::string instead of char* as an argument of getParams
  * fix typo
  * fix doc
  * add docs
  * fix typo
  * separate yaml config for every agent
  * use separate configs for distinct agents
  * remove constructor without arguments, initialize objects inside parameter constructor
  * use AlicaContext parameter constructor
  * use AlicaContext parameter constructor
  * use AlicaContext parameter constructor
  * use AlicaContext parameter constructor
  * remove buildObjects function
  * add getConfig to AlicaEngine
  * return boolean to signal setOption success
  * return false in setOptions if at least one value was not set correctly
  * add noexcept to setOptions, catch InvalidNode exception
  * pass const references to setOptions
  * traverse yaml node iteratively in setOptions
  * remove setOptions helper function
  * use enhanced for loop
  * set getLocalAgent to const
  * remove unnecessary this
  * remove persistentId
  * remove Global from agent configs
  * add subscribe function
  * add getConfigPath
  * use AlicaEngine functions instead of accessing AlicaContext
  * remove getter for AlicaContext
  * add subscribe function with reloadFunctionPtr as a parameter
  * use subscribe with reloadFunctionPtr
  * remove old subscribe function, use new subscribe function
  * remove ConfigChangeListener from AlicaContext
  * remove implementation of ConfigChangeListener interface
  * remove configChangeListener interface
  * pass ptr of reload function via subscribe
  * catch all yaml exceptions, print error msg, simplify currentNode initialization
  * remove updateAgentValues, add setter for defaultRole
  * use functionPtr to update Components, remove unsubscribe function
  * do not create multiple AlicaEngineInfo objects on reload
  * do not regenerate random token on reload, set timeout and defaultrole for local agent on reload
  * add setter for defaultRole
  * update test for changing config values
  * prevent duplicate / outdated CapabilityPairs in localAnnouncement
  * add comment describing why values of yaml node are checked
  * do not store engine ptr locally
  * take configPath as const ref
  * dont pass bool by ref
  * remove old config files
  * remove reference from bool parameter
  * add AlicaContextParams struct
  * add constructor with AlicaContextParams struct as parameter
  * add doc comments
  * add new initConfig function, declare _configRootNode and _localAgentName earlier
  * add new initConfig implementation
  * update AlicaContext constructor
  * use new initConfig function in constructor
  * remove addition of / in initConfig
  * remove initConf call in init function
  * remove old initConfig function
  * declare _configPath earlier
  * add initializer list
  * remove setting _configPath in initConfig
  * initialize _configRootNode and _configPath in initializer list
  * initialize _clock, _communicatior and _idManager in initializer list
  * add _configRootNode member
  * add initConfig to engine
  * add _reloadFunctions member
  * add agentName and configPath to constructor
  * pass agentName and configPath to engine constructor
  * add _configRootNode and _reloadFunctions to initializer list
  * add agentName and configPath to constructor, remove fullConfigPath
  * use new context constructor
  * use new context constructor
  * use new context constructor
  * move config files
  * create engine in initializer list
  * declare _reloadFunctionPtrs earlier
  * rename to setOptions
  * use setOptions
  * add setOptions
  * add _initialized member
  * set initialized to true at the end of init
  * add reloadConfig function
  * use engines reloadConfig when setOption is called
  * change reloadConfig visibility to public
  * store reload functions in engins reloadFunctions vector
  * remove reloadAll function
  * remove subscribe function
  * remove setInitialized
  * use engines subscribe function
  * pass context yaml config node to reloadConfig
  * remove setOption functions
  * remove setOption declaration
  * remove config node from engine
  * remove initialized flag
  * set initialized flag in init function
  * use AlicaContext init function
  * remove reloadFunctions vector from initializer list
  * remove old code
  * remove agentName from engine constructor
  * add basePath to ModelManager constructor
  * set domainConfigFolder in initializer list
  * rename parameter to domainConfigFolder
  * remove getConfigPath function
  * update getConfig comment
  * use const reference for accessing config path
  * print error when context is already initialized
  * add reloadConfig
  * replace lambdas with std::bind
  * replace agentID  parameter with yaml config
  * abort readSelfFromConfig when localAgent has been created before
  * remove test for updating TeamManager component
  * remove setDefaultRole
  * remove setDefaultRole
  * fix allignment
  * remove unnecessary assertion
  * remove terminate and startCommunication call
  * remove redundant test
  * add AlicaTestNotInitializedFixture
  * use AlicaNotInitializedFixture for test
  * atomic setOptions
  * update setOptions doc
  * remove old config files
  * add config as parameter
  * add bool logging
  * rename reloadConfig to reload
  * subscribe to config changes
  * remove const from AlicaEngine ptr
  * add implementation for new AlicaContext constructor
  * use AlicaContext with AlicaContextParams
  * look for config in agent and config dir
  * update comment of constructor
  * use AlicaContextParams for initialization
  * remove old constructor
  * add exception warning to constructor comment
  * add comment to reloadConfig
  * remove configPath and reloadFunctionPtrs
  * fix comments
  * remove initConfig
  * remove clearing capabilities
  * initialize communication with nullptr
  * initialize _roleSet with nullptr
  * add lock for _readyForSync access
  * initialize roleID with 0
  * initialize _priorityDefault with 0
  * set default value of stepEngine to false
  * remove setLocalAgentName
  * remove const from bool parameter
  * use ALICA_WARNING_MSG
  * use << for warning msg
  * set _initialized to false on termination
  * use auto for reload funcion ptr
  * rename _reloadFunctions to _configChangeListenerCBs
  Co-authored-by: Stephan Opfer <StephanOpfer@users.noreply.github.com>
* Merge pull request `#119 <https://github.com/rapyuta-robotics/alica/issues/119>`_ from rapyuta-robotics/revise_alica_test_utility_api
  Revise alica test utility api
* forget some parts
* Merge branch 'rr-devel' into revise_alica_test_utility_api
* - removed agent access methods
* address review comments
* - moved isStateActive API to test UTIL
* Merge branch 'rr-devel' into remove_id_api_from_context
* Merge branch 'rr-devel' into fix_remove_dbg_output
* Merge pull request `#120 <https://github.com/rapyuta-robotics/alica/issues/120>`_ from rapyuta-robotics/abhi_fix_api
  Remove IdentifierConstPtr reference from public api
* Remove IdentifierConstPtr from TestContext
* update comment
* introduced Util class for separating TestContext from Alica internal tests
* - made SuccessOrFailBehaviour default constructable
* - fused success and failure mockup behaviour
* fixed behaviour trigger -> now it is almost redundant
* - cleanup alica_tests with improved TestContext-API
* first step to run tests without getBehaviourPool()
* store work in progress
* Merge pull request `#114 <https://github.com/rapyuta-robotics/alica/issues/114>`_ from rapyuta-robotics/json-plan-format
  Wrong merge...
* Merge pull request `#112 <https://github.com/rapyuta-robotics/alica/issues/112>`_ from rapyuta-robotics/sop_test_framework
  Initial Version of the Alica Test Utilities
* - updated tests to run without engine getter
  - introduced extra getDomainVariable method in TeamManager
* address PR comments:
  - remove I from non-interface class
  - remove engine getter (at first only temporarily to see, if we can live without it)
  - cleanup CMakeList.txt
* - fix dereferenceing unique pointer
* make things more unique
* rename prepareStepping to makeBehaviourEventDriven
* improved debug output
* adapted alica to the improved event_handling package
* - made fail and success behaviour usable via the TestBehaviourCreator
* - refactored test library and improved API
  - adapted alica tests
  - minor improved engine
* Contributors: Abhishek S, Abhishek Sharma, Dale Koenig, Luca Tricerri, Maksim Derbasov, Prajapati-Pawan, Stephan Opfer, bjoernschroeder, bschroeder, dhananjay-patki, mansiVerma26, veerajsk
