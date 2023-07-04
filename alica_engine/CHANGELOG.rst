^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_engine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Log once instead of throwing (`#500 <https://github.com/rapyuta-robotics/alica/issues/500>`_)
  * Log instead of throwing
  * Avoid spamming if stuck for too long
  * Simplified
  * Review comments
  * nit
* Remove old Logger and Logging (`#497 <https://github.com/rapyuta-robotics/alica/issues/497>`_)
  * removed logger
  * removed comments
  * pre-commit
  ---------
  Co-authored-by: Rogerio Fonteles <rogerio.fonteles@moley.com>
* Logging fixes (`#496 <https://github.com/rapyuta-robotics/alica/issues/496>`_)
  * Logging fixes
  - Make AlicaDefaultLogger thread safe since it is the responsibility of
  the logging implementation
  - Set the logger immediately on construction of the context so that it
  can be used to log messages during construction & before init() is
  called. This solves some obsure bugs where the logger was used before
  it was initialized in the context's set api's eg. setTraceFactory or the
  trace factory'sconstructor etc.
  - Provides the ability to set the logger multiple times before the
  context is initialized which is consistent with other set api's
  - Destroy the logger when the context is destroyed
  - Add tests for custom logging & fix the test for default logging
  * Fix compile error in supplementary tests
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
* Ignore unknown agents (`#493 <https://github.com/rapyuta-robotics/alica/issues/493>`_)
  * ignore unknown agents
  * add warning log
  * fix typo
* Add support for running the same plan / behaviour in the same state multiple times (`#487 <https://github.com/rapyuta-robotics/alica/issues/487>`_)
  * pass confAbstractPlanWrapper to engine
  * use keymapping from conf abstract plan wrapper
  * add tests for running the same plans / behaviours multiple times in the same state
  * remove old code
  * assert _wrapper != nullptr before accessing keyMapping
  * remove old code
  * add and fix tests
  * add test for running plans & behs in parallel
  * move tests to SameInParallelTestPlan
  * use make_shared instead of new for creating RunningPlan
  * dont initialize runnableObjects with nullptr
  ---------
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Remove msgs of all deactivated agents (`#488 <https://github.com/rapyuta-robotics/alica/issues/488>`_)
  * remove msgs of all deactivated agents
  * remove deactivated agents vector, fix removing msgs of inactive agents
  * add test for reproducing assertion error
  * step agents once
  * Revert test
  ---------
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
* Custom workflows plan (`#491 <https://github.com/rapyuta-robotics/alica/issues/491>`_)
  * Implementation
  * Move files
  * Include fix
  * Fix file name typo
  * Set success, failure
  * Add tutorials
  * Fix file & namespace name
  * Implement plans
  * Fixes
  * Fix plans
  * Implement populate blackboard
  * working version
  * Unit test
  * Fix parsing
  * Remove unused condition
  * Delete unused file
  * Remove unused function
  * Remove unused files
  * Fix supplementary tests
  * Replace transport workflow with teleport workflow
  Transport workflow does not make much sense for turtles therefore
  replace the tutorial with a teleport workflow instead
  * Fix mapping
* Improve type determination in blackboard impl (`#479 <https://github.com/rapyuta-robotics/alica/issues/479>`_)
  * Improve type determination in blackboard impl
  The type determination method used to default construct a value of the
  given type which meant that -
  1. an unnecessary object would be created just for determining the type
  which could potentially be a costly operation that could have side
  effects depending on the type stored
  2. the default constructor might not be available, so such types would
  not be able to be stored on the blackboard
  3. a variant was constructed from the value & the type determined using
  the variant index. This is a potentially confusing operation because
  the type determination relies on the variant implementation.
  To overcome this, the type is determined at compile time following
  clear steps to determine supported type conversions:
  - ints are promoted to int64 (already existed)
  - uints are promoted to uint64 (already existed)
  - floats are promoted to double (indirectly supported earlier, but
  explicitly added now)
  - the promoted type is then checked to see if it is present in the
  variant types at compile time
  - if not found, the type is treated as std::any
  * Review comments
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
* Remove configuration leftovers (`#485 <https://github.com/rapyuta-robotics/alica/issues/485>`_)
  remove configuration leftovers
* unsubscribe from config change listener on destruction of CycleManage… (`#484 <https://github.com/rapyuta-robotics/alica/issues/484>`_)
  * unsubscribe from config change listener on destruction of CycleManagement
  * add unit test for config subscribe and unsubscribe
  * Use structured binding and range based for loop
  ---------
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Improved error handling & tests in dynamic loading (`#477 <https://github.com/rapyuta-robotics/alica/issues/477>`_)
  * Improved error handling & tests in dynamic loading
  - Throw exceptions instead of crashing if dynamic loading cannot load
  the requested symbol from the given library
  - Add unit tests for these error handling cases
  - Add unit tests for utility function creator
  - Make the destructor of UtilityFunction virtual since it is used
  polymorphically to access the DefaultUtilityFunction
  - Add unit tests for implementation name usage for plans & behaviours
  - Add unit tests for constraint creator
  * Enable tests, fix stack smashing
  There were lots of places where factory methods were returning
  unique_ptrs instead of shared_ptrs causing stack smashing issues
  * Fix more places
  * Small fix
  * Fix alica_tests, unique_ptr->shared_ptr for dynamic loading
  * Review comments
* Removing Configuration (`#469 <https://github.com/rapyuta-robotics/alica/issues/469>`_)
  * Removing Configuration
  * removed Configuration strings
  * Removing files
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Remove master plan flag (`#472 <https://github.com/rapyuta-robotics/alica/issues/472>`_)
  * Remove master plan flag
  * Fix
  * Remove strings
* Remove pluginName (`#473 <https://github.com/rapyuta-robotics/alica/issues/473>`_)
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Value mapping (`#456 <https://github.com/rapyuta-robotics/alica/issues/456>`_)
  * Add const mapping blackboard API
  * Add mapping APIs
  * Read value mapping from yaml file
  * Map value
  * Add missing def
  * Fix some bugs
  * Value mapping tests (`#460 <https://github.com/rapyuta-robotics/alica/issues/460>`_)
  * add value mapping tests
  * fix type when accessing blackboard
  * fix testValueMappingCantParseValue test
  * add HasTestError condition, use globalBlackboard for failing tests
  * print error msg from blackboard on test failure
  * simplify tests
  * replace deprecated impl() call, provide fixture for private access to blackboard
  * Turtlesim with value mapping and implementationName (`#458 <https://github.com/rapyuta-robotics/alica/issues/458>`_)
  * Implementation name in engine
  * Turtlesim with value mapping and implementation name
  * Fix turtle test
  * Go to corner when not in formation
  * Normal shutdown
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
  Co-authored-by: bjoernschroeder <39679198+bjoernschroeder@users.noreply.github.com>
* Remove legacy transitions (`#455 <https://github.com/rapyuta-robotics/alica/issues/455>`_)
  remove LegacyTransitionConditions
* Cleanup debug logs in constraint queries (`#459 <https://github.com/rapyuta-robotics/alica/issues/459>`_)
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
* Remove ROS from Alica Tests (`#361 <https://github.com/rapyuta-robotics/alica/issues/361>`_)
  * Removed ROS dependencies
  * Added CMake Packaging commands
  * fix errors
  * one weird broken test
  * cleanup files/naming
  * Moved AlicaTimer
  * PR feedback
  * fix new tests:
  * addressed comments and fixed simplePlanTestWithLegacyConditions test
  * wip: rebase errors
  * tests changed with ae changing from raw pointer  to unique_ptr
  * wip: changing tests back but failing for now
  * dynamic loading in the correct location
  * update on cmake
  * added lib
  * corrected lib name
  * tests working
  * removed tearDown
  * addressed Dale's comments
  * refactored AlicaTimer
  * most of comments addressed
  * change to sleep logic as requested my Maksim
  * tiny change
  * removed param from getRoleSetName
  * addressed veeraj's comments
  * fixed test
  * creation of another base class to get test folder path
  * added explanation msg for while ((sleep_duration > 0) && _isActive)
  * correction on comment
  * removed turtlesim tests added by mistake
  * addressed comments
  * addressed another Dale's comments
  * changed order of callback call
  ---------
  Co-authored-by: Rogerio Fonteles <rogerio.fonteles@moley.com>
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Minor engine improvements (`#433 <https://github.com/rapyuta-robotics/alica/issues/433>`_)
  * Minor engine improvements to allow fewer declarations of inheriting objects
  * Allow export basic plan
  ---------
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Revert "removing Configuration" (`#434 <https://github.com/rapyuta-robotics/alica/issues/434>`_)
  Revert "removing Configuration (`#428 <https://github.com/rapyuta-robotics/alica/issues/428>`_)"
  This reverts commit 31bc444bb52f30aaba4407fa9d593c77ce88383a.
* removing Configuration (`#428 <https://github.com/rapyuta-robotics/alica/issues/428>`_)
  * removing Configuration
  * Clean-up
  * Review
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Clean up debug logs (`#430 <https://github.com/rapyuta-robotics/alica/issues/430>`_)
  * Clean up debug logs
  * review
* Fix for warnings (`#431 <https://github.com/rapyuta-robotics/alica/issues/431>`_)
* Add unlocked blackboard (`#425 <https://github.com/rapyuta-robotics/alica/issues/425>`_)
  * Add unlocked blackboard
  - Add UnlockedBlackboard class that provides no mutex protection
  - Hide internal blackboard implementation classes in an internal
  namespace
  * Move BlackboardException outside the internal namespace
  * Fix access to blackboard internals
  Prevents warnings on accessing internals i.e. BlackboardImpl class
  in alica & alica tests
  * Add unit test for unlocked blackboard
* Remove friends from Task (`#424 <https://github.com/rapyuta-robotics/alica/issues/424>`_)
  * Remove friends from Task
  * Fix constructors
  ---------
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Fix Alica tracing chars string value handling (`#423 <https://github.com/rapyuta-robotics/alica/issues/423>`_)
  fix tracing chars string value handling
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Cleanup (`#426 <https://github.com/rapyuta-robotics/alica/issues/426>`_)
* Remove references to nonexistent modelfactory (`#427 <https://github.com/rapyuta-robotics/alica/issues/427>`_)
  remove references to nonexistent modelfactory
* Removing defaultValue (`#422 <https://github.com/rapyuta-robotics/alica/issues/422>`_)
* Remove model manager ref from team manager (`#415 <https://github.com/rapyuta-robotics/alica/issues/415>`_)
* Remove precompiler stuff for c++11 (`#416 <https://github.com/rapyuta-robotics/alica/issues/416>`_)
* Remove unused code about idle tasks (`#414 <https://github.com/rapyuta-robotics/alica/issues/414>`_)
  * Remove a couple unused ptrs
  * Fix
  * Remove more IDLE stuff
  * Remove default destructor
* Remove access to deprecated default value (`#406 <https://github.com/rapyuta-robotics/alica/issues/406>`_)
  * Remove access to deprecated default value
  * Review round
* Fix Alica blackboard and tracing initialization order (`#404 <https://github.com/rapyuta-robotics/alica/issues/404>`_)
  fix blackboard and tracing init order
* Thread sanitizer fixes (`#400 <https://github.com/rapyuta-robotics/alica/issues/400>`_)
* Remove engine references from constraintsolver (`#398 <https://github.com/rapyuta-robotics/alica/issues/398>`_)
  * Remove engine ref from ISolver
  * Fix supplementary tests
  * Fix turtlesims
* Fix problems spotted by clang (`#396 <https://github.com/rapyuta-robotics/alica/issues/396>`_)
  * Fix problems spotted by clang
* Turtlesim without code generation (`#384 <https://github.com/rapyuta-robotics/alica/issues/384>`_)
  * Add dynamic constraint & utility creators
  * Remove code gen files
  * Unused params fix: review comments
  * Move condition impl to plan files
  * Redesign state machine
  * WIP
  * First working version
  * Update code gen for creator changes
  * Fix readme
  * Address review comments
  * Protected -> Input keys for wait for trigger keys
* Minor code fixes (`#395 <https://github.com/rapyuta-robotics/alica/issues/395>`_)
  * Minor code fixes
* Fixes to properly free allocated memory at shutdown (`#388 <https://github.com/rapyuta-robotics/alica/issues/388>`_)
  * Memory leak fix
  * Stop communication in dtor
* Add agent id, agent name to global blackboard (`#392 <https://github.com/rapyuta-robotics/alica/issues/392>`_)
  * Add agent id, agent name to global blackboard
  * Make sure agent id is generated
* Check plan hash while checking for duplicate agent (`#391 <https://github.com/rapyuta-robotics/alica/issues/391>`_)
* Cleanup for unused functions (`#387 <https://github.com/rapyuta-robotics/alica/issues/387>`_)
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
* Improve logs (`#378 <https://github.com/rapyuta-robotics/alica/issues/378>`_)
  * WIP
  * More improvements to runnable object tracing & logging
  * Improve turtlesim
  * Remove highly spammy debug logs
  * Remove more spammy logs
  * Remove task assignment logs
  * Address review comments
  * Fix compile error
  * Fix typos
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
* Add support for blackboard types (`#338 <https://github.com/rapyuta-robotics/alica/issues/338>`_)
  * add support for blackboard types, use std::variant instead of std::any in blackboard
  * fix int64_t access
  * add converter to handle get different for variant and any
  * remove convertConst struct
  * reenable initDefaultValues
  * add support for promoting types from source to target blackboard
  * fix monostate issues
  * add support for accessing internal type of std::any values in blackboard
  * fix some review comments
  * add get with non const return value, use if constexpr for type dependent implementation of parser and get, remove yaml implementation for std::any and std::monostate
  * fix timing issue
  * clean up code, add comments, fix some errors
  * Fix tests
  * WIP
  * Cleanup
  * Tests working
  * Fix format
  * Exceptions thrown from get
  * Throw more exceptions
  * Add additional tests for blackboard types PR (`#363 <https://github.com/rapyuta-robotics/alica/issues/363>`_)
  * add bb tests for up and down conversion
  * add UnknownType class to tests, add tests for get exceptions
  * add tests for setting and initialization, add non constructable helper class, rename test fixture for blackboard
  * add set and init test for blackboard
  * use EXPECT_THROW for exception testing
  * - use template for mapping tests
  - use constexpr for type names
  - improve exception tests, dont check exception msg
  - compare UnknownTypes by using an int value, passed during initialization
  - add additional tests for std::any
  * add tests setWithConvertibleType and setWithoutSpecifyingType, use default constructor to create default values of base types
  * activate tests for different types convertible to int64_t
  * format
  * remove old code
  * Set yaml type for blackboard mapping tests
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
  * Remove commented out code
  * Review comments
  * Use set API for mapping from src to target blackboard
  This implies the same kind of type conversions are supported by mapping
  & set API. This additionally avoids the need to suppress warnings in the
  make variant helper & prevents conversions between type categories i.e.
  bool -> double, int -> double etc. but supports conversions within the
  same type category eg. int8_t -> int64_t, float -> double etc
  * Add support for unsigned integer types
  * Address minor review comments
  * Fix format
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Allow using basic types in traces (`#372 <https://github.com/rapyuta-robotics/alica/issues/372>`_)
  * allow using basic types in traces
  * fix formatting and merge errors
  * add kv logs
  * fix formatting
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
* Fix warnings from Wextra and Wpedantic (`#364 <https://github.com/rapyuta-robotics/alica/issues/364>`_)
  * fix wpedantic
  * fix Wextra
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Fix code generation for conditions (`#367 <https://github.com/rapyuta-robotics/alica/issues/367>`_)
  * Fix codegen, fix tests
  * Fix supplementary tests
  * Fix ros1 turtlesim
  * Fix ros2 turtlesim
* Fix for warnings (`#362 <https://github.com/rapyuta-robotics/alica/issues/362>`_)
* Minor cleanup (`#355 <https://github.com/rapyuta-robotics/alica/issues/355>`_)
  * Minor cleanup
  * Fixes
  * Fixes
* Dynamic library loader behaviour/plan/runtimecondition (`#339 <https://github.com/rapyuta-robotics/alica/issues/339>`_)
  * merged from dynamic-library-loader-behaviour-002
  * fix build
  * fix
  * fix
  * fix
  * fix
  * pre-commit
  * missing boost
  * docker fix
  * fix
  * fix
  * fix
  * fix
  * library turtlesim moved
  * alica_turtlesim_library no more a package
  * fix
  * fix
  * fix
  * fix minor on CMake
  * move supplementary test library
  * fix package
  * change name for dynamic load library for unittest
  * fix as PR also Logging
  * fix as PR search library in multiple folders
  * fix format
  * Fix dynamicloading param in turtlesim
  * fix deletion bug
  * added DynamicTransitionConditionCreator step 001
  * fix format
  * Update supplementary/alica_ros_turtlesim/README.md
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  * Update supplementary/alica_ros_turtlesim/alica_turtlesim_library/include/turtle.hpp
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  * minor fix for blocks
  * added transitionconditio to turtlesim
  * fix
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Change Werror policy (`#344 <https://github.com/rapyuta-robotics/alica/issues/344>`_)
  * change werror policy
  * fix for Werror
  * minor
  * modified for structured bindings
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
* Run precommit on all files (`#340 <https://github.com/rapyuta-robotics/alica/issues/340>`_)
* Removed Redundant cmake Libraries (`#336 <https://github.com/rapyuta-robotics/alica/issues/336>`_)
* Removed ROS1 and Boost dependencies (`#337 <https://github.com/rapyuta-robotics/alica/issues/337>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* PR16_second_version - Move xxxRuntimeFactory to PlanBase  (`#328 <https://github.com/rapyuta-robotics/alica/issues/328>`_)
  * move RuntimeBehaviourFactory to PlanBase
  * move RuntimePlanFactory to PlanBase
  * get rid of deprecated step 001
  * get rid of some deprecated
  * minor
  * minor
  * fix warning as error in travis
  * fix minor deprecated
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
* Remove AlicaEngine::this from UtilityFunctions (`#305 <https://github.com/rapyuta-robotics/alica/issues/305>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
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
* Allow setting global context for traces and easy access to TraceFactory (`#324 <https://github.com/rapyuta-robotics/alica/issues/324>`_)
  * Allow setting global context to the traces (`#321 <https://github.com/rapyuta-robotics/alica/issues/321>`_)
  * Allow setting global parent context to the traces
  * Fix formatting and tests
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
  * Fix cherry-pick from 4cb7c4e0da4693da495de21ef1b45961be0b623c
  * fix merge
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
* Improve setup for ALICA logging (`#299 <https://github.com/rapyuta-robotics/alica/issues/299>`_)
  * add IAlicaLogger, add comment to logger
  * add IAlicaLogger, add setter for logger in context, initialize logger when not set
  * replace old logging with new logging
  * add TestLogger
  * create fixture with logging
  * update use of query
  * add test for logger, set log verbosity in config
  * update AlicaRosLogger implementation
  * remove ALICA_DEBUG_MSG macro use, remove access of logger via engine, pass logger to objects via constructor
  * pass logger to RunnableObject, remove use of engine for accessing logger, remove debug include from utilityFunction
  * add logging verbosity to config, update query creation for supplementary tests
  * add logger to utility functions
  * update codegen for using logger with utility functions
  * format
  * move jars to LFS
  * - Change logger into singleton
  - prevent singleton destruction
  - add verbosity level warning when initializing
  - remove logger from constructors
  - move logger files into logging
  * remove logger from generated code
  * update AlicaRosLogger
  * replace prints with logs, format
  * remove unused destructor
  * - support logging with << operator
  - update logging to use <<
  - add logFunctions for each verbosity level
  - remove unused includes & forward declarations
  - revert log in Logging.h to old format
  * format
  * update codegen jars
  * fix review comments
  * remove test changes
  * remove newline after logSpace, check if logging config value exists before parsing
  * remove unused codegen changes
  * add comment to setLogger
  * remove empty line from codegen
  * Logger: alica_tests, supplementary_tests and turtlesim (`#302 <https://github.com/rapyuta-robotics/alica/issues/302>`_)
  * regenerate files
  * update supplementary tests, move generated files to src/test/Expr
  * update turtlesim
  * format
  * fix review comments
  * update turtlesim, supplementary and alica_tests
  * regenerate supplementary_tests, delay logger destruction in multiagent tests
  * reactivate tests, delay logger destruction
  * fix includes
  * - remove logspace from tests
  - update TestLogger for updated interface
  - add log tests to CMake
  - fix includes
  - add logging values to config
  * add logging verbosity to configs, remove delayed singleton deletion, set verbosity to info
  * format
  * set verbosity to info, increase delay in logger test
  * destroy logger after each test, update logger test
  * remove print
  * regenerate code
  * change query to non pointer, remove unused includes
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  * move logger initialization to the top of AlicaContext::init
  * - remove destroy api from logger
  - remove TestLogger
  - move initialization of logger to the top of AlicaContext::init
  * format
  * use SLEEP_UNTIL instead of fix sleep time
  * use AlicaRosLogger in alica_tests
  * - move communication start after engine init
  - dectivate non working tests
  - use default logger in alica_tests
  * - use AlicaRosLogger in tests
  - move communication start in front of engine init
  * add new line to logging/Logging.h, use iterator to access _verbosityStringToVerbosity map in AlicaContext
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
* PR13 - Remove AlicaEngine::this from ExpressionHandler (`#301 <https://github.com/rapyuta-robotics/alica/issues/301>`_)
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
  * remove AlicaEngin::this from teammanager
  * Remove AlicaEngine::this from Logger
  * rebase
  * fix formatting
  * remove AlicaEngin::this from teammanager
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
  * step 002
  * step 004
  * fix code for test
  * try fix test test_alica_condition_plan.cpp
  * step 001
  * RuntimePlanFactory
  * rebase
  * step 002
  * step 004
  * fix code for test
  * try fix test test_alica_condition_plan.cpp
  * step 001
  * remove some deprecated method
  * minor
  * step 002
  * step 004
  * step 001
  * fix
  * rebase
  * fix merge
  * fix merge 002
  * fix merge minor
  * fix as comments in PR
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
* PR12 - Remove AlicaEngine::this from AbstractPlan  (`#300 <https://github.com/rapyuta-robotics/alica/issues/300>`_)
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
  * remove AlicaEngin::this from teammanager
  * Remove AlicaEngine::this from Logger
  * rebase
  * fix formatting
  * remove AlicaEngin::this from teammanager
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
  * step 002
  * step 004
  * fix code for test
  * try fix test test_alica_condition_plan.cpp
  * step 001
  * RuntimePlanFactory
  * rebase
  * step 002
  * step 004
  * fix code for test
  * try fix test test_alica_condition_plan.cpp
  * step 001
  * remove some deprecated method
  * minor
  * fix format
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
  * fix format
  * fix as PR comments
  * fix merge
  * fix merge with rr_revel
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
  Co-authored-by: Prajapati-Pawan <100663745+Prajapati-Pawan@users.noreply.github.com>
  Co-authored-by: bjoernschroeder <39679198+bjoernschroeder@users.noreply.github.com>
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* PR11 - Remove AlicaEngine::this from RuntimePlanFactory  (`#298 <https://github.com/rapyuta-robotics/alica/issues/298>`_)
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
  * remove AlicaEngin::this from teammanager
  * Remove AlicaEngine::this from Logger
  * rebase
  * fix formatting
  * remove AlicaEngin::this from teammanager
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
  * step 002
  * step 004
  * fix code for test
  * try fix test test_alica_condition_plan.cpp
  * step 001
  * RuntimePlanFactory
  * rebase
  * fix format
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
  * fix format
  * fix as PR comments
  * fix comments as PR
  * minor
  * fix as PR
  * fix ar PR comments
  * fix comment as PR
  * fix include
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
  Co-authored-by: Prajapati-Pawan <100663745+Prajapati-Pawan@users.noreply.github.com>
  Co-authored-by: bjoernschroeder <39679198+bjoernschroeder@users.noreply.github.com>
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
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
* Setup tracing after blackboard (`#314 <https://github.com/rapyuta-robotics/alica/issues/314>`_)
  * Setup tracing after blackboard
  Setup the blackboard before tracing so that blackboard values can be
  used in settup up custom tracing
  * Fix flaky test
* PR9 - Remove AlicaEngine::this from PlanBase (`#296 <https://github.com/rapyuta-robotics/alica/issues/296>`_)
  * Update conditions (`#307 <https://github.com/rapyuta-robotics/alica/issues/307>`_)
  * update alica_tests
  * regenerate files
  * update supplementary tests
  * update turtlesim
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  * fix PR comments step 001
  * fix PR comments step002
  * fix PR comments step003
  * fix format
  * fix comments as PR step 004
  * fix comments as PR step 005
  * format fix
  * fix ar PR notify
  * fix comments in PR
  Co-authored-by: bjoernschroeder <39679198+bjoernschroeder@users.noreply.github.com>
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
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
* PR6 - Remove AlicaEngine::this from StaticRoleAssignment (`#289 <https://github.com/rapyuta-robotics/alica/issues/289>`_)
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
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
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
* PR5 - Remove AlicaEngine::this from AuthorityManager (`#288 <https://github.com/rapyuta-robotics/alica/issues/288>`_)
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
  * fix format
  * added reload for config
  * fix test
  * regression fix
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
* PR4 - Removing AlicaEngine::this from VariableSyncModule (`#287 <https://github.com/rapyuta-robotics/alica/issues/287>`_)
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
  * added reload for config
  * fix const in timer
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* PR3 - Removing AlicaEngine::this from SyncModule  (`#285 <https://github.com/rapyuta-robotics/alica/issues/285>`_)
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
  * added reload for config
  * minor
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
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
* Fix alica tests (`#275 <https://github.com/rapyuta-robotics/alica/issues/275>`_)
  * Fix plans, add failure handling plans
  * Fix tests
  * Use alica_tests as package name for code gen
  * Fix format
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* Remove alica engine dependency from ModelManager (`#276 <https://github.com/rapyuta-robotics/alica/issues/276>`_)
  Co-authored-by: Abhishek Sharma <abhishek.sharma@rapyuta-robotics.com>
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
* More cleanup (`#273 <https://github.com/rapyuta-robotics/alica/issues/273>`_)
  * Rename RunnableObjectNew to RunnableObject
  * Remove Scheduler
* Creators fixes (`#264 <https://github.com/rapyuta-robotics/alica/issues/264>`_)
  * Creators fixes
  * Ensure stepEngine does not deadlock
  * Various fixes with creators
  * More fix
  * Revert mistaken change in tag
* Fix double locking of fast path events mutex (`#267 <https://github.com/rapyuta-robotics/alica/issues/267>`_)
  While processing fast path events the PlanBase thread can activate
  plans & behaviours on a RunningPlan. Activation involves calling init
  & run (first call only) which now happen on the same thread. init/run
  can set success/failure which adds more fast path events. Therefore
  the fast path event mutex can be locked twice. This change avoids the
  double locking by making a copy of the fast past events before
  processing them.
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
* Remove/Deprecate blackboard edit functions (`#245 <https://github.com/rapyuta-robotics/alica/issues/245>`_)
* Fix memory issues in alica (`#243 <https://github.com/rapyuta-robotics/alica/issues/243>`_)
  * Init values
  * Destruct plans in reverse order
* Blackboard json (`#239 <https://github.com/rapyuta-robotics/alica/issues/239>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Multiple alica instances (`#241 <https://github.com/rapyuta-robotics/alica/issues/241>`_)
  * ignore agentAnnouncements of agents using different rolesets
  * store masterPlanId in planHash, fix planHash overflow, remove unnecessary check
  * use uint64 for planhashes in messages, store planhash in agentQuery as uint64_t, set planhashes in assignment tests
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* add string include (`#236 <https://github.com/rapyuta-robotics/alica/issues/236>`_)
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
* Remove alica engine dependency from AlicaCommunication (`#224 <https://github.com/rapyuta-robotics/alica/issues/224>`_)
  * initial test
  * working version with all fptrs
  * fix
  * code format
  * revert varsyncmodule changes
  * refactor to prevent forward declaration warning (facing error)
  * fix
  * remove headers from i alicacomm
  * update alica dummy communication
  * missed
  * refactor to make less verbose
  * use pointer to handlers
  * take agentid as arg in sendroleswitch
  * don't use pointer to handlers
  * updates in function parameter constness
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
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
* Creation on nullptr check for PlanContext (`#221 <https://github.com/rapyuta-robotics/alica/issues/221>`_)
  * add isValid method for nullptr check, add unittest
  * format
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* Blackboard fix (`#215 <https://github.com/rapyuta-robotics/alica/issues/215>`_)
  * Blackboard fix
  * Fix case
  * Fix case
  * Explicitly bind this instead of all
  * Add missing capture
  * pr comment
* Replace Variant with std::variant (`#198 <https://github.com/rapyuta-robotics/alica/issues/198>`_)
  * use std::variant
  * update tests
  * cleanup
  * cleanup
  * string instead of byte array
  * update ros msg conversion
  * fix
  * pr feedback
  * remove void ptr
  * code format
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
* made getTrace public (`#210 <https://github.com/rapyuta-robotics/alica/issues/210>`_)
  * made getTrace public
  * made get trace protected inside BasicPlan and BasicBehaviour
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Destroy trace after onTermination (`#212 <https://github.com/rapyuta-robotics/alica/issues/212>`_)
* Cherry pick tracing improvements from v0.9.2 (`#206 <https://github.com/rapyuta-robotics/alica/issues/206>`_)
  * Cherry pick tracing improvements from v0.9.2
  Also includes a bug fix that was introduced when RunnableObject was
  refactored: if init is not executed then terminate should not be
  executed.
  * Fix test
  * Address review comments
  - Add docs
  - capture tracing context by value to prevent dangling references
  * Make TracingType public
  * Move the context getter instead of copy
  * Update alica_engine/include/engine/RunnableObject.h
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
  * Add comment
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Parameter based data flow (`#187 <https://github.com/rapyuta-robotics/alica/issues/187>`_)
  * New simple blackboard
  * Remove blackboard from engine
  * Create base classes for parameter setting
  * Find appropriate conf wrapper if needs params
  * Store plan attachments and look them up at correct time
  * Working version
  * Revert "Remove blackboard from engine"
  This reverts commit 948ef87aa7b0061e88ced955fb7e2ade69cd0e6d.
  * Properly load tests
  * Main tests pass
  * Update turtlesim code
  * Regenerate and fix supplementary tests
  * Pass down blackboard when possible
  * Remove ByteArray
  * Get rid of plan attachment creator again
  * Fix compilation
  * BlackBoard pointer usage
  * Get rid of default attachment
  * Regen supp
  * Main tests pass
  * Fix alica tests
  * regenerate other stuff
  * Minor fix
  * Mutex for blackboard
  * some review comments
  * Redesign blackboard
  * Allow cheat access
  * Test fixes
  * Delete some copy assignment/constructors
  * BlackBoard -> Blackboard
  * Fix filename
  * Fix
  * Add missing const
  * Format
* TaskAssignment Tracing (`#178 <https://github.com/rapyuta-robotics/alica/issues/178>`_)
  * implement TestTracing
  * add fixture for testTracing
  * create plan for testing, regenerate
  * fixes after rebase
  * use size_t for vector size comparison
  * regenerate files
  * create plan for testing, regenerate
  * add traces for ta changes
  * add fixtures for tracing
  * add tracing for all ta changes
  * add unittest for task assignment tracing
  * add support for disabled traces
  * replace identifier
  * add getName
  * remove iTrigger
  * use basicPlan trace for TA tracing
  * add oldUtility, newUtility and numberOfAgents to trace log, fix test
  * fix TestTracing
  * update unittest for additional task assignment tracing information
  * check parent-child relationship of traces
  * do TATracing on scheduler thread
  * change Roleset
  * fix conflicts
  * move notifyAssignmentChange after addChildren
  * remove getTrace
  * move check for TraceFactory to notifyTaskAssignmentChange, remove checking for trace on main thread
  * update ta trace log
* Remove the need of setting a worldmodel (`#201 <https://github.com/rapyuta-robotics/alica/issues/201>`_)
  * remove assertion for wm, init wm as nullptr
  * remove empty wms
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
* Remove duplicate code (`#185 <https://github.com/rapyuta-robotics/alica/issues/185>`_)
  * Base class for BasicPlan and BasicBehaviour
  * Finish BasicPlan and BasicBehaviour
  * empty
  * Move implementations into header
  * Reorder instructions
  * Move cancel job
  * Better access contorl to internals
* Release v0.9.2 (`#176 <https://github.com/rapyuta-robotics/alica/issues/176>`_)
  * implement planPool
  * start implementation of tracing
  * implement IAlicaTrace and IAlicaTraceFactory, add tracing for behaviours init/terminate
  * replace tracecontext with string, change methods to pure virtual
  * add getter for traceFactory
  * replace getter for traceContext with getter for trace
  * use setLog for init/terminate traces, use factory to create traces
  * remove amr_interfaces dependency
  * add default constructor, make create pure virtual
  * return ptr to traceFactory
  * return ptr to traceFactory
  * add trace tag for run methods, check if trace / tracefactory exists before usage
  * add name to basicPlan, use name for tag
  * remove member variable and constructors
  * destroy traces after setting terminate tag
  * add comment to getTraceFactory method
  * use setLog, log run once, make getTrace() protected
  * Fixes in creating traces
  - Create traces in the execution context during init.
  - Provide the option to enable/disable tracing
  * Check if trace factory is setting before using it
  * Set basicplan name
  * Bug fixes
  - check if basic plan exists before accessing the trace context
  - if no parent is found create a root trace
  - remove hack to create basic plan
  * Fix alica engine termination
  Do the following in order to correctly terminate the engine:
  1. Stop the plan base thread. This prevents any more changes to the
  running plan.
  2. Deactivate the running plan in plan base stop(). This ensures the
  plans/behaviours are scheduled for termination in the correct order.
  3. Stop all behaviours & plans that may be running. There should not
  be any since (2) ensures this taken care of
  4. Execute all pending non repeatable jobs in the scheduler thread.
  This ensures the plan/behaviour init/terminates are actually executed.
  5. Stop the scheduler thread
  Add a trace log that indicates if the trace is for a behaviour or a
  plan.
  Note: the master plan trace is not reported because the master plan does
  not have a corresponding basic plan attached to it. This is a bug &
  the reason for it is because there is no ConfAbstractPlanWrapper for the
  master plan due to which it is not in the plan pool. This has to be fixed
  in a separate PR.
  * Bring back scheduler null check in engine terminate
  * Avoid unused variable warning
  * Create BasicPlan object for the master plan
  There is no ConfAbstractPlanWrapper corresponding to the master plan.
  Therefore get the master plan details by calling getPlans() on the
  PlanRepository which includes the master plan details.
  * Fix ci
  Co-authored-by: bjoernschroeder <bschroederprogramming@gmail.com>
* implement planPool (`#165 <https://github.com/rapyuta-robotics/alica/issues/165>`_)
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
* Track RunningPlan in beh, plan's exec context + optimize (`#174 <https://github.com/rapyuta-robotics/alica/issues/174>`_)
  The running plan under which the behaviour or plan is executing was not
  tracked (execution context). Instead only the running plan under which
  the behaviour or plan was started i.e. signal context was tracked. This
  meant that it was possible for the behaviour to access a destroyed
  running plan object.
  Optimization: Skip calling init/terminate when the behaviour/plan's
  execution is out of context with the signal from the main thread.
  Improve comments
* Bas beh sched engine (`#147 <https://github.com/rapyuta-robotics/alica/issues/147>`_)
  Plan init/run/terminate + plan/behaviour scheduling + web based designer
  1. Introduce init, run, terminate functionality for plans (analogous to their counterparts for behaviours)
  2. Introduce ordered scheduling of plan & behaviour init, run & terminate methods. The engine now provides the following guarantees:
  a. The init method of the parent plan will be executed before the init of any of its child plans/behaviours
  b. The run method of the plan/behaviour will be executed after the init of that plan/behaviour (was also the case earlier) & the terminate will execute after the run/init method is executed. From a user's perspective, the engine does not guarantee that the run will be called (for eg. run could be skipped if the behaviour is terminated immediately), however the current implementation does.
  c. The terminate of the child plans/behaviours will be executed before the terminate of the parent plan
  d. When a transition is made from state A to state B, terminate for all plans/behaviours in state A will be called before calling init for all plans/behaviours in state B
  e. The init & terminate for plans/behaviours in a single state will be executed sequentially but no order is guaranteed. This implies that it is not possible for init/terminates to run in parallel
  These guarantees are enforced through scheduling the inits & terminates on a single scheduler thread. The run method is scheduled by the init & the terminate waits for the run to complete.
  3. The engine uses a multithreaded timer pool to execute the run methods, so run's can be executed in parallel as before. The timer pool is taken an a parameter to the engine so users are free to provide any implementation. The default implementation is the one in alica-supplementary that is based on ros. This also means that there is no separate thread for each behaviour as was the case earlier, thereby dramatically reducing the number of threads used by the engine
  4. The engine is now compatible with the new plan designer. The plan layouts in alica_tests are done in the new web based plan designer & the code is regenerated as per the new code generation which generates the init/run/terminate for plans
  Co-authored-by: Stephan Opfer <StephanOpfer@users.noreply.github.com>
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
* Merge pull request `#150 <https://github.com/rapyuta-robotics/alica/issues/150>`_ from rapyuta-robotics/vsk_cherry-pick_fix_build
  fix build (`#144 <https://github.com/rapyuta-robotics/alica/issues/144>`_)
* fix build (`#144 <https://github.com/rapyuta-robotics/alica/issues/144>`_)
* Merge pull request `#136 <https://github.com/rapyuta-robotics/alica/issues/136>`_ from rapyuta-robotics/feature/use_current_assignments
  Use Existing Assignments in Utility Summands
* - extended the alica engine's USummand interface to also use the old assignment
  - adapted all existing Summans in alica_tests
  - wrote a new test that uses the extended interface
* Merge pull request `#135 <https://github.com/rapyuta-robotics/alica/issues/135>`_ from rapyuta-robotics/fix_missing_header
  added missing header, that made constraintsolver package not compilin…
* Merge branch 'fix_missing_header' of github.com:rapyuta-robotics/alica into fix_missing_header
* Merge branch 'rr-devel' into fix_missing_header
* added missing header, that made constraintsolver package not compiling successful
* Merge pull request `#123 <https://github.com/rapyuta-robotics/alica/issues/123>`_ from rapyuta-robotics/doxygen_integration
  Doxygen integration
* Merge branch 'rr-devel' into doxygen_integration
* Merge branch 'rr-devel' into gh-pages
* Merge pull request `#133 <https://github.com/rapyuta-robotics/alica/issues/133>`_ from rapyuta-robotics/fix_parsing_quantifiers
  replace "ALL" with "all"
* Merge branch 'rr-devel' into fix_parsing_quantifiers
* replace "ALL" with "all"
* Merge pull request `#131 <https://github.com/rapyuta-robotics/alica/issues/131>`_ from rapyuta-robotics/improve_comment_on_id
  Improve Doxygen Comment
* removed part about invalid id
* - removed unused deprecated config
  - improved comment on id parameter in AlicaContextParams
* Merge pull request `#129 <https://github.com/rapyuta-robotics/alica/issues/129>`_ from rapyuta-robotics/fix_path_composition_in_model_manager
  - fixed path composition in AlicaContext and ModelManager
* Merge branch 'rr-devel' into fix_path_composition_in_model_manager
* remove debug comment
* - fixed path composition in AlicaContext and ModelManager
* Merge pull request `#128 <https://github.com/rapyuta-robotics/alica/issues/128>`_ from rapyuta-robotics/fix_coverity
  - no reference for agentid in teammanger
* - no reference for agentid in teammanger
* Merge pull request `#127 <https://github.com/rapyuta-robotics/alica/issues/127>`_ from rapyuta-robotics/bas_alica_warning_msg
  use ALICA_WARNING_MSG instead of cerr
* use ALICA_WARNING_MSG instead of cerr
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
* - added some comment on how to use it
* change flag
* - fix doxyfile and moved out of ignored folder
* - initial setup of doxygen with CMAKE
* Merge pull request `#119 <https://github.com/rapyuta-robotics/alica/issues/119>`_ from rapyuta-robotics/revise_alica_test_utility_api
  Revise alica test utility api
* Merge branch 'rr-devel' into revise_alica_test_utility_api
* Merge pull request `#116 <https://github.com/rapyuta-robotics/alica/issues/116>`_ from rapyuta-robotics/remove_id_api_from_context
  removed forwarding id API
* address review comments
* - moved isStateActive API to test UTIL
* Merge branch 'rr-devel' into remove_id_api_from_context
* Merge pull request `#121 <https://github.com/rapyuta-robotics/alica/issues/121>`_ from rapyuta-robotics/fix_remove_dbg_output
  - remove spamming debug output
* Merge branch 'rr-devel' into fix_remove_dbg_output
* - remove spamming debug output
* Merge pull request `#120 <https://github.com/rapyuta-robotics/alica/issues/120>`_ from rapyuta-robotics/abhi_fix_api
  Remove IdentifierConstPtr reference from public api
* Remove IdentifierConstPtr reference from public api
* clang format BasicBehaviour
* - cleanup alica_tests with improved TestContext-API
* first step to run tests without getBehaviourPool()
* removed forwarding id API
* store work in progress
* Merge pull request `#114 <https://github.com/rapyuta-robotics/alica/issues/114>`_ from rapyuta-robotics/json-plan-format
  Wrong merge...
* Merge pull request `#112 <https://github.com/rapyuta-robotics/alica/issues/112>`_ from rapyuta-robotics/sop_test_framework
  Initial Version of the Alica Test Utilities
* Merge pull request `#106 <https://github.com/rapyuta-robotics/alica/issues/106>`_ from rapyuta-robotics/json-plan-format
  Json plan format
* Merge branch 'sop_test_framework' of github.com:rapyuta-robotics/alica into sop_test_framework
* Merge branch 'json-plan-format' into sop_test_framework
* - updated tests to run without engine getter
  - introduced extra getDomainVariable method in TeamManager
* - removed comments from merging
* Merge branch 'rr-devel' into json-plan-format
* adapted alica to the improved event_handling package
* Merge pull request `#79 <https://github.com/rapyuta-robotics/alica/issues/79>`_ from rapyuta-robotics/bb_race_fix
  Fix several race conditions in BasicBehaviour
* - refactored test library and improved API
  - adapted alica tests
  - minor improved engine
* - implemented step mode for behaviours
* added friend declaration towards AlicaTestSupportUtility
  added dependency towards alica_test_support
* changed comment, as not all behaviour creators are autogenerated
* Merge pull request `#103 <https://github.com/rapyuta-robotics/alica/issues/103>`_ from rapyuta-robotics/repair-tests
  Repair tests
* Merge pull request `#107 <https://github.com/rapyuta-robotics/alica/issues/107>`_ from rapyuta-robotics/featureCommonConfs
  Feature common confs
* fixed merge issues
* Merge branch 'v0.9.0' into featureCommonConfs
* Merge branch 'repair-tests' into featureCommonConfs
* - add missing brace
* Update AlicaContext.h
  added message to deprecated method declaration
* - changed c++ std in cmake lists
  - added std::cout in TeamManager constructor for having the final local agent id in the log files
  - made getRobotName deprecated in alica context
  - changed variable sync module from pointer to unique_ptr in engine
* - implemented unit test for configrations on behaviours
  - adapted running plan for storing configurations for plantypes and plans, too
* - added states into sub plans (fix for invalid assignment problem in task assignment)
  - adapted behaviourpool to init basic behaviours with configurations
* - added plan for testing configurations (WIP)
  - fixed debugoutput include in header file of query (makes problems elsewhere)
* - implemented Configuration, ConfAbstractPlanWrapper
  - removed artifact from PlanningProblem class (some forward declaration and header files)
  - extended parser of the engine for the ConfAbstractWrapper entries in json files
* - just formatting
* Merge pull request `#104 <https://github.com/rapyuta-robotics/alica/issues/104>`_ from rapyuta-robotics/vsk_agentID_param
  Take AgentID argument as an object instead of a pointer
* Take a const reference to agent id instead of the object itself
* Take AgentID argument as an object instead of a pointer
* Merge pull request `#84 <https://github.com/rapyuta-robotics/alica/issues/84>`_ from rapyuta-robotics/addAgentIdv0.9.0
  Add agent idv0.9.0
* - made agentID a const AgentID*
  - fixed segfault in the list of initialisers of the AlicaEngine constructor (horrible line for TeamManager gets removed, when IDManager is owned by AlicaContext in future)
* changed constructor's agent parameter to AgentID* and copied the given AgentID* via the _agentIDManager
* - removed debug output from AlicaContext
  - improved debug output in Synchronisation Process
  - fixed multiple entries in SyncRow's ReceivedBy list
  - fixed segfault/freez/undefined behaviour after a successfull synchronisation (deletion of SynchProcesses is now always done in SyncModule.tick())
* - minor adaption of debug output
* - put debug output into canonical alica_debug_message() macros
  - put a mutex in the destructor of SyncProcess
* - tried to fix synchronisation module: Test works, but synch protocol/module seems to be completely broken
* - renamed robotID to agentID
  - improved debug output
* - fix all but synchronisation test
* - fixed initialisation dependency
  - renamed method in alica context
* - resolved circular header dependency between AlicaEngine.h and VariableSyncModule.h
  - standardised include guard for classes under src/engine/expressionhandler/
* - made the alica_tests compile under json format
  - current issue: initialisation order of ALICA Engine -> things get messy, need a more proper solution here...
* - fix copy paste error
* - fix initialisation order for unit tests
* - removed unnecessary includes
  - reduced timeout for running alica_tests successful from 600 to 20 seconds
* Merge pull request `#86 <https://github.com/rapyuta-robotics/alica/issues/86>`_ from dasys-lab/integrate-auto-discovery
  New JSON Plan Format
* - fix for essentials include
* - updated CMake version
* - restored alica_tests
* Merge branch 'rapyuta-robotics-v0.9.0' into integrate-auto-discovery
* Merge branch 'v0.9.0' of https://github.com/rapyuta-robotics/alica into rapyuta-robotics-v0.9.0
* Removed ROS_INFO statements and ros.h
* added changes after testing todo: code cleanup
* added a functionality to acquire agentid and default arg to constructor declarations
* - removed debug output
* - implemented parsing of terminal states
* - removed PlanWriter and tinyxml
* - removed artifacts
* - removed artifacts from behaviour conf
  - improved runningplan output in case of behaviour
* Merge branch 'rr-devel' into bb_race_fix
* Merge pull request `#83 <https://github.com/rapyuta-robotics/alica/issues/83>`_ from rapyuta-robotics/catkin_install_build
  Add install targets for catkin install build
* - removed empty line
* - made evaluation of transition preconditions more robust
* Merge branch 'rr-devel' into catkin_install_build
* Remove unused header TestBase.h
* Merge pull request `#75 <https://github.com/rapyuta-robotics/alica/issues/75>`_ from rapyuta-robotics/wb_store_id_option
  Wb store id option
* - added IdManager getter
* Add install targets for catkin install build
* Casting, id type, naming & missing include fixes
  1. Change unsigned long long to uint64_t
  2. Use numeric_limits<uint64_t>::max() to get the max value
  3. Use static_cast instead of c-style cast to invoke conversion function
  4. Change persist_id to persistId as per naming convention
  5. Use uint64_t for id in test cases, otherwise it will fail
  6. Remove extra inclusion of header <random>
* remove some warnings
* impletemtation of the PersistID option
* Fix several race conditions in BasicBehaviour
  Since several flag variables like _started, _callInit, _success, _failure
  & _running are accessed & modified by both the calling thread (ALICA main
  plan base thread) & the behaviour thread, it can lead to several race
  conditions & synchronization issues which are fixed in this commit:
  1. Problem: behaviour thread can wait on condition variable even after
  receving start signal as the main thread does not modify _running under
  the _runLoopMutex.
  Solution: use the _runLoopMutex whenever modifying the signal state.
  2. Problem: isSuccess() & isFailure() can both return true at same time
  & _failure, _success can cause race conditions due to no mutex/atomic protection.
  Solution: maintain behaviour result status in atomic enum variable
  which can be in unknown, failure or success state, not both.
  3. Problem: If behaviour is stopped & started in quick succession, the
  stop may never been seen leading to onTermination() not being called.
  Solution: Maintain behaviour state, signal state & if stop was called
  while behaviour is running user code. Call onTermination() every time
  behaviour is stopped.
  4. Problem: behaviour can set success or failure in initialiseParameters().
  Solution: behaviour can set success or failure only in run().
  5. Cleanup code flow especially for triggered behaviour. Behaviour waits
  to receive start signal, then moves to run state (where it waits for trigger
  /stop in case of triggered behaviour). Then it terminates when stop is called.
* Merge pull request `#82 <https://github.com/rapyuta-robotics/alica/issues/82>`_ from rapyuta-robotics/ctx_clock_api
  Provide api in AlicaContext to use a custom clock
* Provide api in AlicaContext to use a custom clock
  A custom clock is useful for simulations & testing in order to control the
  speed of the simulation. This clock can be set in the AlicaContext and will
  be used by the engine for time-based work.
* Merge pull request `#80 <https://github.com/rapyuta-robotics/alica/issues/80>`_ from rapyuta-robotics/ctx_is_state_active
  Change getCurrentState() api to isStateActive() in AlicaContext
* Make isStateActiveHelper() a static method of AlicaContext
  Make isStateActiveHelper() a static method of AlicaContext instead of
  a standalone function to improve code structure.
* Change getCurrentState() api to isStateActive() in AlicaContext
  getCurrentState() returns only the active state in the deepest alica plan
  tree node. Other active states cannot be known using this method. Therefore
  change the api to isStateActive() which can check if a given state is active
  in the entire current plan tree.
* Merge pull request `#78 <https://github.com/rapyuta-robotics/alica/issues/78>`_ from rapyuta-robotics/v1.0.0
  Merge branch v1.0.0 into rr-devel
* Merge branch 'rr-devel' into v1.0.0
* Merge pull request `#77 <https://github.com/rapyuta-robotics/alica/issues/77>`_ from rapyuta-robotics/bugfix/missing_header
  add missing header file
* add missing header file
* Merge pull request `#76 <https://github.com/rapyuta-robotics/alica/issues/76>`_ from rapyuta-robotics/assignment_crash
  Fix assignment failure and crash
* Fix assignment failure and crash
* - removed model factory and parser
  - finished new parster factories (hopefully - except roles stuff)
* Merge pull request `#72 <https://github.com/rapyuta-robotics/alica/issues/72>`_ from hendrik-skubch/minor_fixes
  Minor fixes
* Merge pull request `#70 <https://github.com/rapyuta-robotics/alica/issues/70>`_ from rapyuta-robotics/version_bump
  Minor cleanup and version change to 0.9.0
* Update TaskAssignmentProblem.cpp
  travis bump
* - switched to new id package
* - added parsing of behaviour configurations
  -> missing initialisation of basicbehaviour with parameters in behaviour pool
  -> adaption of state content parsing in plans (now there are configurations not behaviours)
* clean up old toString method
* fix race condition in Query.cpp, fix missing include in TeamManager
* Minor cleanup and version change to 0.9.0
* Merge pull request `#69 <https://github.com/rapyuta-robotics/alica/issues/69>`_ from rapyuta-robotics/localconf_improve
  Improve Local.conf to not depend on host name and add new context api's
* Improve Local.conf to not depend on host name and add new context api's
* Add api's to step engine and get local agent info from context
* Merge pull request `#1 <https://github.com/rapyuta-robotics/alica/issues/1>`_ from rapyuta-robotics/rr-devel
  Update from RR
* Merge pull request `#68 <https://github.com/rapyuta-robotics/alica/issues/68>`_ from rapyuta-robotics/coverity_fix
  Missed default initialization for members
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* - extended role switch, so that alica communication proxies don't need a real engine for sending
  - made the receiving callbacks testing on nullptr for engine object -> print to console if null
* Missed default initialization for members
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* Merge pull request `#66 <https://github.com/rapyuta-robotics/alica/issues/66>`_ from rapyuta-robotics/auto_discovery
  Enable auto discovery of agents
* - made engine a little more robust against missing communication and clock module
* Change role to role id in message
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* - moved AgentIDConstPtr
* Reduce scope
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* Fix compilation error
* Find if duplicate agents exist on network with same id
* Add discovery test case and fix review comments
* - made rule stuff work
* Fix initialization crash for variablesync
* Fix test cases
* - factories
  - remove old stuff (wip)
* Enable auto discovery of agents
* - model manager attaches references, creates template variables (seems to work so far, waiting for rolesets)
* - further factories: TaskRepo, Task, Behaviour, PlanType, Synchronisation (improved)
  - implemented parse loop in ModelManager
  - improved alica model
* - started to have filesParsed and filesToParse
  - problem with list of references (maybe fix in Plan Designer)
* - made some parsing safety measurements
* - rectified some parts of the alica model (there never was a synctransition)
  - finished parsing plans (without testing so far)
* - Factories for PreCondition(wip), Transition, State, Plan(wip), EntryPoint, VariableBinding(wip)
* - fix get Referenced Id
  - fix missing ep insertion
* fixed some errors for null values in strings
* - introduced new folder modelmanagement
  - began to code the parsing part of the ModelManager
* - initial commit for reworking parser/writer
* - some stuff for entry points
* - hmpf
* - initial stuff for parsing yaml via idiomatic way of yaml-cpp
* Merge pull request `#65 <https://github.com/rapyuta-robotics/alica/issues/65>`_ from rapyuta-robotics/fix/crash-on-role-not-found
  Do not prevent an agent from being removed due to state protection timer
* test for interrupting communication between robots
* allow modifying assignment protection time
* allow using a different clock in alica
* Do not prevent an agent from being removed due to state protection timer
* Add missing #include <functional> needed for g++7 (`#62 <https://github.com/rapyuta-robotics/alica/issues/62>`_)
* Merge pull request `#60 <https://github.com/rapyuta-robotics/alica/issues/60>`_ from rapyuta-robotics/new_api
  New api for alica
* Rolled back to static init function for running plan
* Added const getters in engine
* supple -> essential
* Fix GetSeed crash
* Merge branch 'master' into newPD
* Add warning comment for engine members declaration order dependencies
* Fix duplicate communicator startup and change function ordering
* Add version getter
* Fixed test refactoring issues
* Refactor tests to use context
* Merge pull request `#14 <https://github.com/rapyuta-robotics/alica/issues/14>`_ from rapyuta-robotics/rr_to_daisys
  Refactoring into 3 repo
* Merge branch 'new_api' of github.com:rapyuta-robotics/alica into new_api
* Revert "move to c++14, clean cmakelist files"
  This reverts commit 4ee14c88f6e460921f704980f718dd56265ecde3.
* Refactor Engine to work with new context
* Merge branch 'rr-devel' into new_api
* Switch to C++14 (`#59 <https://github.com/rapyuta-robotics/alica/issues/59>`_)
  * move to c++14, clean cmakelist files
  * fix eclipse cpp version
* move to c++14, clean cmakelist files
* Add Alica Context api class
* Merge pull request `#58 <https://github.com/rapyuta-robotics/alica/issues/58>`_ from rapyuta-robotics/uncache_creators
  Remove caching of alica creators
* Merge branch 'rr-devel' into uncache_creators
* Merge pull request `#57 <https://github.com/rapyuta-robotics/alica/issues/57>`_ from rapyuta-robotics/hs_add_some_success_test_accessors
  Some more detailed tests for success needed for a project
* Further expressionhandler cleanup
* Remove caching of alica creators
* Merge branch 'rr-devel' into hs_add_some_success_test_accessors
* Merge pull request `#56 <https://github.com/rapyuta-robotics/alica/issues/56>`_ from rapyuta-robotics/fix_naming_convention
  Follow naming convention in engine
* Changed filestoparse to vector
* Merge branch 'fix_naming_convention' of github.com:rapyuta-robotics/alica into fix_naming_convention
* Changed unnecessary protected access modifiers to private
* reduce scope
* --typo
* Some more naming fix and 'this' removal
* Final naming convention fix for engine
* Further optimization in code and review comment fixes
* fix confusing capital
* added success tests that only test the local agent's aspect
* Follow naming convention in engine
* Merge pull request `#55 <https://github.com/rapyuta-robotics/alica/issues/55>`_ from rapyuta-robotics/system_config_fix
  Cleanup system config caching
* Cleanup system config caching
* Coverity fix (`#53 <https://github.com/rapyuta-robotics/alica/issues/53>`_)
* handle basic behaviour termination cleanly (`#52 <https://github.com/rapyuta-robotics/alica/issues/52>`_)
  * handle basic behaviour termination cleanly
  * simplify
* fix unused code that sneaked through repo refactoring (`#51 <https://github.com/rapyuta-robotics/alica/issues/51>`_)
* Fix agent results sync issue (`#50 <https://github.com/rapyuta-robotics/alica/issues/50>`_)
  * Fix agent results sync issue
  * Take care of review comments
* - hacks finished
* - added hack for pkvr
* Merge pull request `#48 <https://github.com/rapyuta-robotics/alica/issues/48>`_ from rapyuta-robotics/new_alica
  WIP to move to 3 repo structure
* Further sync with new supplementary and essentials
* Renamed supplementary to essentials namespace for moved components
* startet to change tinyxml to yaml
* Minor bugfix (`#46 <https://github.com/rapyuta-robotics/alica/issues/46>`_)
  * make sure flags are set before notifications go out
  * duplicate include
  * improved handling of deepest node pointer
  * wait for query to be finished before testing result
* - reformat
* if fast equality checks are allowed, use pointer address for hashing (`#45 <https://github.com/rapyuta-robotics/alica/issues/45>`_)
* Add callback to behavior for notifying of being terminated (`#44 <https://github.com/rapyuta-robotics/alica/issues/44>`_)
  * call onTermination whenever a behavior is stopped
  * typo
  * addtional comments
  * call before taking mutex
  * typo
  Co-Authored-By: hendrik-skubch <hendrik.skubch@rapyuta-robotics.com>
  * typo
  Co-Authored-By: hendrik-skubch <hendrik.skubch@rapyuta-robotics.com>
* - renamed getVariableByName into getVariable and the BasicBehaviour now calls Behaviour::getVariable(..) instead of implemting the same again
* - removed unused setVariables method (variables are added directly of friend classes like ModelFactory and ExpressionHandler)
* - some fixes according to hendrik thx
* Merge branch 'master' into newPD
* Merge pull request `#11 <https://github.com/rapyuta-robotics/alica/issues/11>`_ from rapyuta-robotics/rr_to_upstream
  Refactored Assignment and RunningPlan interfaces
* Merge branch 'master' into newPD
* - removed SigFault.h because it does not work anymore
* Hs rework failure handling (`#40 <https://github.com/rapyuta-robotics/alica/issues/40>`_)
  * WIP
  * better debug output
  * WIP 2
  * WIP 3
  * remove state collection
  * utility function overhaul
  * WIP
  * cleanup
  * move IAssignment
  * WIP
  * rename taskAssignment to reduce name confusion
  * rename
  * renaming
  * WIP
  * wip more
  * Remove obsolete assignmentcollection
  * refactor AgentIDConstPtr
  * new AgentIDConstPtr
  * more WIP
  * fixes
  * various fixes
  * compile!
  * remove obsolete lock
  * missing ctrl+s
  * basic lifetime management for running plan, thrad safe interface for behavior code
  * cleanup in basicbehaviour, properly fix racecondition
  * clean up
  * add test
  * adapt alica ros com
  * optimization
  * adapt tests to changed api
  * fix
  * fixes
  * fixes
  * fixes
  * adapt unit test to changed api
  * remove debug output
  * fixes
  * fix trigger test: a behavior does not own it's trigger
  * fixes
  * fixes
  * fixes
  * fixes
  * fixes
  * fixes
  * fix
  * fix
  * fixes
  * polish & removal of race condition
  * clean
  * fix allowIdling
  * clean
  * clean package xml, add hash to AgentIDConstPtr
  * check ids
  * coverity fixes
  * must take lock to refresh contextInRun pointer, otherwise context is not update upon state transitions to similar states
  * clean unnecessary parameter, fix deadlock
  * fix race condition
  * Test that a behaviour is not locked by switching states repeatedly.
  * remove rolesetdir from engine constructor, reactivate one unit test
  * clean up whitespaces
  * style cleaning
  * reduce code duplicaiton in tests
  * more removal of code duplication
  * Update QueryBehaviour1.h
  * remove code duplication
  * simplification
  * simplification
  * remove include
  * convinience method
  * convinience method
  * fix
  * more convenience methods
  * fix
  * one more convenience function
  * clean up
  * remove agentsAvailable from RunningPlan and provide view to active agents
  * more consistent naming and removal of getActiveAgentProperties
  * clean up
  * tighten class Agent
  * tighten CycleManager
  * minor polish
  * clean cycleManager
  * simpify allocationdifference
  * improve debug output
  * remove unnecessary virtual
  * some additional debug values
  * more debug info
  * remove engine pointer from successMarks
  * clean successMarks
  * add unique ptr
  * clean duplication of id
  * rename
  * cleaning
  * wip
  * cache runtime condition result
  * add missing method, wait for parent plan to react if possible
  * fix test & add another
  * fix issue in alica logger not using config logfolder
  * test behavior failure too
  * formatting
  * cppcheck fix
  * formatting
  * cppcheck
  * cpp check
  * fix bad merge
  * +const
  * switch from exception to assert
* WIP: Fixes from Manipulation Demo (`#39 <https://github.com/rapyuta-robotics/alica/issues/39>`_)
  * fix bad usage of longs
  * more longs
  * more longs gone
  * fix long in parser
  * remove longs
  * add missing NULL
  * one more NULL
  * fix issue with exceeding runtime warning
* Hot fixes (`#38 <https://github.com/rapyuta-robotics/alica/issues/38>`_)
  * fix output issue
  * Fix bug in isSuccessful()
  * no bitwise and...
  * fix state protection...again
  * fix recursiveUpdateAssignment
* Fix runningPlan lifetime issue (`#37 <https://github.com/rapyuta-robotics/alica/issues/37>`_)
  * fix issue with all agents iterator
  * - retired nodes are not part of the plan tree anymore.
  -  fix accidental access of deleted parent
  * stop plan evaluation on retirement
* Further simplifications to RunningPlan (`#34 <https://github.com/rapyuta-robotics/alica/issues/34>`_)
  * WIP
  * better debug output
  * WIP 2
  * WIP 3
  * remove state collection
  * utility function overhaul
  * WIP
  * cleanup
  * move IAssignment
  * WIP
  * rename taskAssignment to reduce name confusion
  * rename
  * renaming
  * WIP
  * wip more
  * Remove obsolete assignmentcollection
  * refactor AgentIDConstPtr
  * new AgentIDConstPtr
  * more WIP
  * fixes
  * various fixes
  * compile!
  * remove obsolete lock
  * missing ctrl+s
  * basic lifetime management for running plan, thrad safe interface for behavior code
  * cleanup in basicbehaviour, properly fix racecondition
  * clean up
  * add test
  * adapt alica ros com
  * optimization
  * adapt tests to changed api
  * fix
  * fixes
  * fixes
  * fixes
  * adapt unit test to changed api
  * remove debug output
  * fixes
  * fix trigger test: a behavior does not own it's trigger
  * fixes
  * fixes
  * fixes
  * fixes
  * fixes
  * fixes
  * fix
  * fix
  * fixes
  * polish & removal of race condition
  * clean
  * fix allowIdling
  * clean
  * clean package xml, add hash to AgentIDConstPtr
  * check ids
  * coverity fixes
  * must take lock to refresh contextInRun pointer, otherwise context is not update upon state transitions to similar states
  * clean unnecessary parameter, fix deadlock
  * fix race condition
  * Test that a behaviour is not locked by switching states repeatedly.
  * remove rolesetdir from engine constructor, reactivate one unit test
  * clean up whitespaces
  * style cleaning
  * reduce code duplicaiton in tests
  * more removal of code duplication
  * Update QueryBehaviour1.h
  * remove code duplication
  * simplification
  * simplification
  * remove include
  * convinience method
  * convinience method
  * fix
  * more convenience methods
  * fix
  * one more convenience function
  * clean up
  * remove agentsAvailable from RunningPlan and provide view to active agents
  * more consistent naming and removal of getActiveAgentProperties
  * clean up
  * tighten class Agent
  * tighten CycleManager
  * minor polish
  * clean cycleManager
  * simpify allocationdifference
  * improve debug output
  * remove unnecessary virtual
  * some additional debug values
  * more debug info
  * remove engine pointer from successMarks
  * clean successMarks
  * add unique ptr
  * clean duplication of id
  * rename
  * cleaning
  * make sure all status check does not lose meaning before PAlloc
  * allow access to engine by child behaviors
  * fix merge
  * fix merge
  * remove blank line
  * more consistent naming & reduce code duplication in tests
* Refactor RunningPlan & Assignment data structures (`#30 <https://github.com/rapyuta-robotics/alica/issues/30>`_)
  * WIP
  * better debug output
  * WIP 2
  * WIP 3
  * remove state collection
  * utility function overhaul
  * WIP
  * cleanup
  * move IAssignment
  * WIP
  * rename taskAssignment to reduce name confusion
  * rename
  * renaming
  * WIP
  * wip more
  * Remove obsolete assignmentcollection
  * refactor AgentIDConstPtr
  * new AgentIDConstPtr
  * more WIP
  * fixes
  * various fixes
  * compile!
  * remove obsolete lock
  * missing ctrl+s
  * basic lifetime management for running plan, thrad safe interface for behavior code
  * cleanup in basicbehaviour, properly fix racecondition
  * clean up
  * add test
  * adapt alica ros com
  * optimization
  * adapt tests to changed api
  * fix
  * fixes
  * fixes
  * fixes
  * adapt unit test to changed api
  * remove debug output
  * fixes
  * fix trigger test: a behavior does not own it's trigger
  * fixes
  * fixes
  * fixes
  * fixes
  * fixes
  * fixes
  * fix
  * fix
  * fixes
  * polish & removal of race condition
  * clean
  * fix allowIdling
  * clean
  * clean package xml, add hash to AgentIDConstPtr
  * check ids
  * coverity fixes
  * must take lock to refresh contextInRun pointer, otherwise context is not update upon state transitions to similar states
  * clean unnecessary parameter, fix deadlock
  * fix race condition
  * Test that a behaviour is not locked by switching states repeatedly.
  * remove rolesetdir from engine constructor, reactivate one unit test
  * clean up whitespaces
  * style cleaning
  * reduce code duplicaiton in tests
  * more removal of code duplication
  * Update QueryBehaviour1.h
  * remove code duplication
  * simplification
  * simplification
  * remove include
  * convinience method
  * convinience method
  * fix
  * more convenience methods
  * fix
  * one more convenience function
  * clean up
  * bug fix
  * adress review comments
  * address some review comments
  * remove one lock
  * fuix due to explicit operator
  * fix minor issue with operators
  * add missing includes
  * missing return
* Fix issue with state start time (`#35 <https://github.com/rapyuta-robotics/alica/issues/35>`_)
  * only limit agents if state has been inhabited for awhile
  * accidental checkin
  * accidental checkin
* temporary using std::string for generated DomainBehaviour (`#32 <https://github.com/rapyuta-robotics/alica/issues/32>`_)
* add exception handling to PlanParser (`#29 <https://github.com/rapyuta-robotics/alica/issues/29>`_)
* fix to problems with autogeneration (`#31 <https://github.com/rapyuta-robotics/alica/issues/31>`_)
* catch runtime errors (`#28 <https://github.com/rapyuta-robotics/alica/issues/28>`_)
* better debug output (`#27 <https://github.com/rapyuta-robotics/alica/issues/27>`_)
* Merge pull request `#10 <https://github.com/rapyuta-robotics/alica/issues/10>`_ from rapyuta-robotics/rr_to_upstream
  Rewrite Autodiff, improve solver interface
* Coverity fixes & removal of namespace usings in headers (`#25 <https://github.com/rapyuta-robotics/alica/issues/25>`_)
  * add common compiler configs
  * add dependencies to common config
  * formatting
  * separate solver interface from main engine, use solvercontext object, relinquish ownership on solvervariables
  * unify representation of intervals
  * fix one memory leak (coverity 274744)
  * remove warnings & fix memory leak
  * nousing namespace
  * get rid of questionable and unused hash function
  * rework SolverVariable treatment
  * update tests
  * Cleaning & updating tests
  * fix missing initialization
  * additional sanity check
  * fixes for query algorithm with new solvervariable
  * warnings as errors & remove debug output
  * fix domain variable to solver variable mapping & unit test
  * remove unused variable
  * clean up + add simple solvercontext
  * move interval to solver interface
  * get rid of separate range representation in problemdescriptor, tie it to autodiff variables
  * Update ProblemDescriptor.h
  add missing const
  * move max interval to autodiff
  * additional features for Interval
  * removal of using namespace std, some coverity fixes
  * header cleaning & coverity fix
  * coverity fixes
  * remove namespace std from headers
  * remove redundant indirection
  * add necessary usings for autogenerated code
  * more const
  * review comments
  * fix bad merge
  * review comments
* Merge pull request `#9 <https://github.com/rapyuta-robotics/alica/issues/9>`_ from rapyuta-robotics/hs_improve_query
  Improvements to query API
* Better Autodiff and separated solver interface (`#23 <https://github.com/rapyuta-robotics/alica/issues/23>`_)
  * add common compiler configs
  * add dependencies to common config
  * formatting
  * separate solver interface from main engine, use solvercontext object, relinquish ownership on solvervariables
  * unify representation of intervals
  * fix one memory leak (coverity 274744)
  * remove warnings & fix memory leak
  * nousing namespace
  * get rid of questionable and unused hash function
  * rework SolverVariable treatment
  * update tests
  * Cleaning & updating tests
  * fix missing initialization
  * additional sanity check
  * fixes for query algorithm with new solvervariable
  * warnings as errors & remove debug output
  * fix domain variable to solver variable mapping & unit test
  * remove unused variable
  * clean up + add simple solvercontext
  * move interval to solver interface
  * get rid of separate range representation in problemdescriptor, tie it to autodiff variables
  * Update ProblemDescriptor.h
  add missing const
  * move max interval to autodiff
  * additional features for Interval
  * review comments
  * add a unit tests for intervals
  * rename interval methods
  * fix unittest
  * add back constexpr compatible assert
* Some fixes (`#24 <https://github.com/rapyuta-robotics/alica/issues/24>`_)
  * remove unused warnings
  * initialize members
* Fixes warnings & coverity issue on UtilityFunction (`#22 <https://github.com/rapyuta-robotics/alica/issues/22>`_)
  * remove warnings & fix memory leak
  * nousing namespace
  * get rid of questionable and unused hash function
* Add a common config package (`#21 <https://github.com/rapyuta-robotics/alica/issues/21>`_)
  * add common compiler configs
  * add dependencies to common config
  * formatting
* Fix minor bugs found with coverity (`#19 <https://github.com/rapyuta-robotics/alica/issues/19>`_)
  * - test
  * Delete LICENSE
  For Hendrik ;-)
  * - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
  * Simplify the query interface
  * Fix test_alica_problem_composition
  * fix task_assignment test
  * remove debug flag
  * fix wrong filter in synctalk
  * Fix to fast path event + test for success from behavior
  * fix copy&paste issue
  * clean up
  * formatting changes
  * first simplification iteration on RunningPlan
  * initial check-in, WIP
  * added .clang_format file
  * Update test_alica_authority.cpp
  That is more obvious.
  * - removed merge artifacts
  * copied stuff from old alica_test to alica_tests
  * Compiler error fixes, simplify ignoring agents (`#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_)
  * Compiler error fixes, simplify ignoring agents
  * remove vscode and add to gitignore
  * added clang format file
  * moved coding guidelines into style folder
  * iteration 2
  * Update CodingGuidelines.md
  Merging the carpe-noctem-cassel coding guide with the one here in alica
  * Update CodingGuidelines.md
  update version
  * Update CodingGuidelines.md
  moved general things upwards + changes namespace examples
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * - tryed to fix wrong naming for alica_test(s)
  - removed old test package
  * - fixed nameing
  * Update CodingGuidelines.md
  - moved pragma once into header section
  - added remark to relative includes
  * Update CodingGuidelines.md
  * more refactoring
  * model constification -> engine compiles
  * style update
  * remove accidental log file
  * clean up
  * better whitespaces
  * fix bad merge
  * format update
  * update to compile supplementary and tests
  * fix remaining compiler issues in tests
  * fix comment issue
  * fixing missing return
  * more errors mkay
  * test improvement after fixing subvariable bug
  * fix planwriter issues
  * clean planningProblem and destinationPath
  * clean capabilityDefinitionSet
  * remove unused RoleUsage
  * cleaning
  * beautification and removal of unnecessary new
  * fix missing sort predicate, add test
  * cleaning & removing some memory allocations
  * reorder header
  * wip
  * fix extra endif
  * beautification
  * caps ftw
  * consistency
  * review comments
  * wip
  * use proper comparison in sorting
  * Initial integration of Variant
  * fix typo
  * address review comments
  * some fixes
  * remove unnecessary setter
  * - Refactored the way solvers are addressed
  - Made Query work with the new solver interface
  - Clean AlicaEngine.h somewhat
  * simplifications
  * add blackboard and hash function
  * fixes & use new interface in tests
  * raise warning level
  * test compile
  * fix test
  * minor fix
  * Variant tests
  * add blackboard test
  * test for variable syncing, clean up and additional static assert
  * const override
  * improve BBIdent comparison operators
  * review comments
  * Update SuccessMarks.h
  makes no sense
  * refactoring query algorithm, WIP checkin
  * cleaning
  * align clang format
  * rename Sets to Grps
  * update types.h
  * fix rename
  * fix issues with clang format file
  * WIP
  * Fix compiler errors
  * rename
  * fix range accessors
  * fix unit tests
  * fix renaming
  * fix bad merge
  * clean assignment & fix bug in forallagents
  * Query API should only use const RunningPlan
  * remove unused functions
  * generalize getVariableByName
  * fix missing sovlervars incase no problempart is involved
  * fix wrong id check
  * bug fixes
  * add variable handling test
  * remove warnings
  * make sure agents know each other in the test first
  * fix condition
  * fix crash bug in synchronization
  * give verbose info
  * increase test timeout
  * minor fixes and give more time to behavior trigger test
  * Prevent buffer overflow
  * address PR comments
  * remove redundant code, switch to editRbots in assignment
  * adress coverity `#274754 <https://github.com/rapyuta-robotics/alica/issues/274754>`_
  * add shutdown call to dtor if needed (coverity `#274751 <https://github.com/rapyuta-robotics/alica/issues/274751>`_)
  * coverity `#24799 <https://github.com/rapyuta-robotics/alica/issues/24799>`_
  * coverity `#274743 <https://github.com/rapyuta-robotics/alica/issues/274743>`_
  * coverity `#274797 <https://github.com/rapyuta-robotics/alica/issues/274797>`_
  * fix branch selection for coverity build trigger
* Merge branch 'ab_fix_bugs_found_with_coverity' into hs_improve_query
* Improve query API (`#15 <https://github.com/rapyuta-robotics/alica/issues/15>`_)
  * - test
  * Delete LICENSE
  For Hendrik ;-)
  * - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
  * Simplify the query interface
  * Fix test_alica_problem_composition
  * fix task_assignment test
  * remove debug flag
  * fix wrong filter in synctalk
  * Fix to fast path event + test for success from behavior
  * fix copy&paste issue
  * clean up
  * formatting changes
  * first simplification iteration on RunningPlan
  * initial check-in, WIP
  * added .clang_format file
  * Update test_alica_authority.cpp
  That is more obvious.
  * - removed merge artifacts
  * copied stuff from old alica_test to alica_tests
  * Compiler error fixes, simplify ignoring agents (`#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_)
  * Compiler error fixes, simplify ignoring agents
  * remove vscode and add to gitignore
  * added clang format file
  * moved coding guidelines into style folder
  * iteration 2
  * Update CodingGuidelines.md
  Merging the carpe-noctem-cassel coding guide with the one here in alica
  * Update CodingGuidelines.md
  update version
  * Update CodingGuidelines.md
  moved general things upwards + changes namespace examples
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * - tryed to fix wrong naming for alica_test(s)
  - removed old test package
  * - fixed nameing
  * Update CodingGuidelines.md
  - moved pragma once into header section
  - added remark to relative includes
  * Update CodingGuidelines.md
  * more refactoring
  * model constification -> engine compiles
  * style update
  * remove accidental log file
  * clean up
  * better whitespaces
  * fix bad merge
  * format update
  * update to compile supplementary and tests
  * fix remaining compiler issues in tests
  * fix comment issue
  * fixing missing return
  * more errors mkay
  * test improvement after fixing subvariable bug
  * fix planwriter issues
  * clean planningProblem and destinationPath
  * clean capabilityDefinitionSet
  * remove unused RoleUsage
  * cleaning
  * beautification and removal of unnecessary new
  * fix missing sort predicate, add test
  * cleaning & removing some memory allocations
  * reorder header
  * wip
  * fix extra endif
  * beautification
  * caps ftw
  * consistency
  * review comments
  * wip
  * use proper comparison in sorting
  * Initial integration of Variant
  * fix typo
  * address review comments
  * some fixes
  * remove unnecessary setter
  * - Refactored the way solvers are addressed
  - Made Query work with the new solver interface
  - Clean AlicaEngine.h somewhat
  * simplifications
  * add blackboard and hash function
  * fixes & use new interface in tests
  * raise warning level
  * test compile
  * fix test
  * minor fix
  * Variant tests
  * add blackboard test
  * test for variable syncing, clean up and additional static assert
  * const override
  * improve BBIdent comparison operators
  * review comments
  * Update SuccessMarks.h
  makes no sense
  * refactoring query algorithm, WIP checkin
  * cleaning
  * align clang format
  * rename Sets to Grps
  * update types.h
  * fix rename
  * fix issues with clang format file
  * WIP
  * Fix compiler errors
  * rename
  * fix range accessors
  * fix unit tests
  * fix renaming
  * fix bad merge
  * clean assignment & fix bug in forallagents
  * Query API should only use const RunningPlan
  * remove unused functions
  * generalize getVariableByName
  * fix missing sovlervars incase no problempart is involved
  * fix wrong id check
  * bug fixes
  * add variable handling test
  * remove warnings
  * make sure agents know each other in the test first
  * fix condition
  * fix crash bug in synchronization
  * give verbose info
  * increase test timeout
  * minor fixes and give more time to behavior trigger test
  * address PR comments
  * quick fix
* Merge pull request `#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_ from carpe-noctem-cassel/robotid
  Robotid
* coverity `#274797 <https://github.com/rapyuta-robotics/alica/issues/274797>`_
* coverity `#274743 <https://github.com/rapyuta-robotics/alica/issues/274743>`_
* coverity `#24799 <https://github.com/rapyuta-robotics/alica/issues/24799>`_
* add shutdown call to dtor if needed (coverity `#274751 <https://github.com/rapyuta-robotics/alica/issues/274751>`_)
* adress coverity `#274754 <https://github.com/rapyuta-robotics/alica/issues/274754>`_
* Merge remote-tracking branch 'origin/hs_improve_query' into ab_fix_bugs_found_with_coverity
* remove redundant code, switch to editRbots in assignment
* address PR comments
* Prevent buffer overflow
* - first steps towards new plan designer (especially without BehaviourConfigurations)
* minor fixes and give more time to behavior trigger test
* fix crash bug in synchronization
* remove warnings
* bug fixes
* fix wrong id check
* fix missing sovlervars incase no problempart is involved
* generalize getVariableByName
* remove unused functions
* Query API should only use const RunningPlan
* clean assignment & fix bug in forallagents
* fix bad merge
* fix renaming
* Merge branch 'hs_constify_model' into hs_improve_query
  # Conflicts:
  #	alica_engine/include/engine/Types.h
  #	alica_engine/include/engine/collections/StateCollection.h
  #	alica_engine/include/engine/collections/SuccessMarks.h
  #	alica_engine/include/engine/constraintmodul/ISolver.h
  #	alica_engine/include/engine/constraintmodul/ProblemPart.h
  #	alica_engine/include/engine/constraintmodul/Query.h
  #	alica_engine/include/engine/constraintmodul/ResultEntry.h
  #	alica_engine/include/engine/model/Condition.h
  #	alica_engine/include/engine/model/ForallAgents.h
  #	alica_engine/include/engine/model/Plan.h
  #	alica_engine/include/engine/model/PlanningProblem.h
  #	alica_engine/include/engine/model/Quantifier.h
  #	alica_engine/include/engine/model/SyncTransition.h
  #	alica_engine/src/engine/constraintmodul/ProblemPart.cpp
  #	alica_engine/src/engine/constraintmodul/Query.cpp
  #	alica_engine/src/engine/constraintmodul/ResultEntry.cpp
  #	alica_engine/src/engine/model/Condition.cpp
  #	alica_engine/src/engine/model/ForallAgents.cpp
  #	alica_tests/autogenerated/include/ConstraintTestPlanDummySolver.h
  #	alica_tests/autogenerated/src/ConstraintTestPlanDummySolver.cpp
  #	alica_tests/src/test/test_statecollection.cpp
* fix unit tests
* rename
* Fix compiler errors
* Merge pull request `#7 <https://github.com/rapyuta-robotics/alica/issues/7>`_ from rapyuta-robotics/hs_constify_model
  Constify the Alica model
* WIP
* fix rename
* update types.h
* rename Sets to Grps
* Merge branch 'rr-devel' into ab_travis_ci_fix
* cleaning
* refactoring query algorithm, WIP checkin
* Update SuccessMarks.h
  makes no sense
* Improved Solver Interface Pass `#1 <https://github.com/rapyuta-robotics/alica/issues/1>`_ (`#13 <https://github.com/rapyuta-robotics/alica/issues/13>`_)
  * - test
  * Delete LICENSE
  For Hendrik ;-)
  * - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
  * Simplify the query interface
  * Fix test_alica_problem_composition
  * fix task_assignment test
  * remove debug flag
  * fix wrong filter in synctalk
  * Fix to fast path event + test for success from behavior
  * fix copy&paste issue
  * clean up
  * formatting changes
  * first simplification iteration on RunningPlan
  * initial check-in, WIP
  * added .clang_format file
  * Update test_alica_authority.cpp
  That is more obvious.
  * - removed merge artifacts
  * copied stuff from old alica_test to alica_tests
  * Compiler error fixes, simplify ignoring agents (`#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_)
  * Compiler error fixes, simplify ignoring agents
  * remove vscode and add to gitignore
  * added clang format file
  * moved coding guidelines into style folder
  * iteration 2
  * Update CodingGuidelines.md
  Merging the carpe-noctem-cassel coding guide with the one here in alica
  * Update CodingGuidelines.md
  update version
  * Update CodingGuidelines.md
  moved general things upwards + changes namespace examples
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * - tryed to fix wrong naming for alica_test(s)
  - removed old test package
  * - fixed nameing
  * Update CodingGuidelines.md
  - moved pragma once into header section
  - added remark to relative includes
  * Update CodingGuidelines.md
  * more refactoring
  * model constification -> engine compiles
  * style update
  * remove accidental log file
  * clean up
  * better whitespaces
  * fix bad merge
  * format update
  * update to compile supplementary and tests
  * fix remaining compiler issues in tests
  * fix comment issue
  * fixing missing return
  * more errors mkay
  * test improvement after fixing subvariable bug
  * fix planwriter issues
  * clean planningProblem and destinationPath
  * clean capabilityDefinitionSet
  * remove unused RoleUsage
  * cleaning
  * beautification and removal of unnecessary new
  * fix missing sort predicate, add test
  * cleaning & removing some memory allocations
  * reorder header
  * wip
  * fix extra endif
  * beautification
  * caps ftw
  * consistency
  * review comments
  * wip
  * use proper comparison in sorting
  * Initial integration of Variant
  * fix typo
  * address review comments
  * some fixes
  * remove unnecessary setter
  * - Refactored the way solvers are addressed
  - Made Query work with the new solver interface
  - Clean AlicaEngine.h somewhat
  * simplifications
  * add blackboard and hash function
  * fixes & use new interface in tests
  * raise warning level
  * test compile
  * fix test
  * minor fix
  * Variant tests
  * add blackboard test
  * test for variable syncing, clean up and additional static assert
  * const override
  * improve BBIdent comparison operators
  * review comments
* review comments
* improve BBIdent comparison operators
* Merge branch 'rr-devel' into ab_travis_ci_fix
* Merge remote-tracking branch 'origin/rr-devel' into hs_solving_reloaded
  # Conflicts:
  #	alica_engine/include/engine/AlicaEngine.h
  #	alica_engine/include/engine/constraintmodul/ResultEntry.h
  #	alica_engine/include/engine/constraintmodul/VariableSyncModule.h
  #	alica_engine/src/engine/AlicaEngine.cpp
  #	alica_engine/src/engine/constraintmodul/ResultEntry.cpp
  #	alica_engine/src/engine/constraintmodul/VariableSyncModule.cpp
  #	alica_tests/src/test/test_alica_init_shutdown.cpp
  #	alica_tests/src/test/test_alica_problem_composition.cpp
* Refactor AlicaTime and AlicaClock (`#12 <https://github.com/rapyuta-robotics/alica/issues/12>`_)
  * - test
  * Delete LICENSE
  For Hendrik ;-)
  * - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
  * Simplify the query interface
  * Fix test_alica_problem_composition
  * fix task_assignment test
  * remove debug flag
  * fix wrong filter in synctalk
  * Fix to fast path event + test for success from behavior
  * fix copy&paste issue
  * clean up
  * formatting changes
  * first simplification iteration on RunningPlan
  * initial check-in, WIP
  * added .clang_format file
  * Update test_alica_authority.cpp
  That is more obvious.
  * - removed merge artifacts
  * copied stuff from old alica_test to alica_tests
  * Compiler error fixes, simplify ignoring agents (`#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_)
  * Compiler error fixes, simplify ignoring agents
  * remove vscode and add to gitignore
  * added clang format file
  * moved coding guidelines into style folder
  * iteration 2
  * Update CodingGuidelines.md
  Merging the carpe-noctem-cassel coding guide with the one here in alica
  * Update CodingGuidelines.md
  update version
  * Update CodingGuidelines.md
  moved general things upwards + changes namespace examples
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * - tryed to fix wrong naming for alica_test(s)
  - removed old test package
  * - fixed nameing
  * Update CodingGuidelines.md
  - moved pragma once into header section
  - added remark to relative includes
  * Update CodingGuidelines.md
  * WIP: refactoring AlicaTime and AlicaClock
  * more refactoring
  * model constification -> engine compiles
  * constexpr AlicaTime and improved AlicaClock
  * keep non-trivial constructor private
  * style update
  * remove accidental log file
  * clean up
  * better whitespaces
  * fix bad merge
  * format update
  * update to compile supplementary and tests
  * AlicaClock: simplify engine setup, add a const
  * fix remaining compiler issues in tests
  * fix comment issue
  * Begin to clean up all code involving AlicaTime
  * fixing missing return
  * more errors mkay
  * test improvement after fixing subvariable bug
  * Continued refactoring of  AlicaTime
  * fix planwriter issues
  * clean planningProblem and destinationPath
  * clean capabilityDefinitionSet
  * remove unused RoleUsage
  * More refactoring
  * cleaning
  * beautification and removal of unnecessary new
  * fix missing sort predicate, add test
  * cleaning & removing some memory allocations
  * reorder header
  * More refactoring
  * Get rid of merge conflict artifact
  * Refactor updates
  * Refactoring
  * Refactor AlicaTime
  * Add operators, unit test, fixes to timings
  * Update CMakeLists.txt
  * Update Alica_Base.md
  * Update Alica_Base.md
  * Update AlicaEngine.cpp
  * Update AlicaEngine.cpp
  * Update test_alica_authority.cpp
  * Update test_alica_behaviourtrigger.cpp
  * Update test_alica_condition_plan.cpp
  * Update test_alica_condition_plan.cpp
  * Update test_alica_condition_plantype.cpp
  * Update test_alica_engine_behavior_pool_init.cpp
  * Update test_alica_engine_plan_parser.cpp
  * Update test_alica_gsolver_plan.cpp
  * Update test_alica_init_shutdown.cpp
  * Update test_alica_multi_agent_plan.cpp
  * Update test_alica_problem_composition.cpp
  * Update test_alica_ros_proxy.cpp
  * Update test_alica_simple_plan.cpp
  * Update test_alica_sync_transition.cpp
  * Update test_success_spam.cpp
  * Update test_alica_condition_plantype.cpp
  * fix error in test with setting clock to wrong AlicaEngine instance
  * Test fractional time input, got rid of dependence on thread timing
* const override
* test for variable syncing, clean up and additional static assert
* Merge remote-tracking branch 'origin/rr-devel' into hs_solving_reloaded
  # Conflicts:
  #	alica_engine/CMakeLists.txt
  #	alica_engine/include/engine/Types.h
  #	alica_engine/include/engine/USummand.h
  #	alica_engine/include/engine/constraintmodul/ISolver.h
  #	alica_engine/include/engine/constraintmodul/IVariableSyncModule.h
  #	alica_engine/include/engine/constraintmodul/Query.h
  #	alica_engine/include/engine/constraintmodul/ResultEntry.h
  #	alica_engine/include/engine/constraintmodul/VariableSyncModule.h
  #	alica_engine/include/engine/planselector/PartialAssignment.h
  #	alica_engine/src/engine/AlicaEngine.cpp
  #	alica_engine/src/engine/constraintmodul/Query.cpp
  #	alica_engine/src/engine/constraintmodul/ResultEntry.cpp
  #	alica_engine/src/engine/constraintmodul/VariableSyncModule.cpp
  #	alica_engine/src/engine/parser/ModelFactory.cpp
  #	alica_engine/src/engine/planselector/PlanSelector.cpp
  #	alica_engine/src/engine/planselector/TaskAssignment.cpp
  #	alica_tests/autogenerated/include/ConstraintTestPlanDummySolver.h
  #	alica_tests/autogenerated/include/Plans/Behaviour/ConstraintUsingBehaviour.h
  #	alica_tests/autogenerated/src/ConstraintTestPlanDummySolver.cpp
  #	alica_tests/autogenerated/src/Plans/Behaviour/ConstraintUsingBehaviour.cpp
  #	alica_tests/src/test/test_alica_condition_plan.cpp
* add blackboard test
* Variant tests
* raise warning level
* fixes & use new interface in tests
* add blackboard and hash function
* simplifications
* - Refactored the way solvers are addressed
  - Made Query work with the new solver interface
  - Clean AlicaEngine.h somewhat
* Merge branch 'hs_constify_model' into hs_solving_reloaded
* Constify model (`#11 <https://github.com/rapyuta-robotics/alica/issues/11>`_)
  * - test
  * Delete LICENSE
  For Hendrik ;-)
  * - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
  * Simplify the query interface
  * Fix test_alica_problem_composition
  * fix task_assignment test
  * remove debug flag
  * fix wrong filter in synctalk
  * Fix to fast path event + test for success from behavior
  * fix copy&paste issue
  * clean up
  * formatting changes
  * first simplification iteration on RunningPlan
  * initial check-in, WIP
  * added .clang_format file
  * Update test_alica_authority.cpp
  That is more obvious.
  * - removed merge artifacts
  * copied stuff from old alica_test to alica_tests
  * Compiler error fixes, simplify ignoring agents (`#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_)
  * Compiler error fixes, simplify ignoring agents
  * remove vscode and add to gitignore
  * added clang format file
  * moved coding guidelines into style folder
  * iteration 2
  * Update CodingGuidelines.md
  Merging the carpe-noctem-cassel coding guide with the one here in alica
  * Update CodingGuidelines.md
  update version
  * Update CodingGuidelines.md
  moved general things upwards + changes namespace examples
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * - tryed to fix wrong naming for alica_test(s)
  - removed old test package
  * - fixed nameing
  * Update CodingGuidelines.md
  - moved pragma once into header section
  - added remark to relative includes
  * Update CodingGuidelines.md
  * more refactoring
  * model constification -> engine compiles
  * style update
  * remove accidental log file
  * clean up
  * better whitespaces
  * fix bad merge
  * format update
  * update to compile supplementary and tests
  * fix remaining compiler issues in tests
  * fix comment issue
  * fixing missing return
  * more errors mkay
  * test improvement after fixing subvariable bug
  * fix planwriter issues
  * clean planningProblem and destinationPath
  * clean capabilityDefinitionSet
  * remove unused RoleUsage
  * cleaning
  * beautification and removal of unnecessary new
  * fix missing sort predicate, add test
  * cleaning & removing some memory allocations
  * reorder header
  * beautification
  * caps ftw
  * consistency
  * review comments
  * use proper comparison in sorting
  * address review comments
  * remove unnecessary setter
* remove unnecessary setter
* some fixes
* address review comments
* Initial integration of Variant
* Merge branch 'hs_constify_model' into hs_solving_reloaded
* use proper comparison in sorting
* wip
* review comments
* consistency
* caps ftw
* beautification
* fix extra endif
* wip
* reorder header
* cleaning & removing some memory allocations
* fix missing sort predicate, add test
* beautification and removal of unnecessary new
* cleaning
* Merge remote-tracking branch 'origin/rr-devel' into hs_constify_model
  # Conflicts:
  #	alica_engine/include/engine/BasicBehaviour.h
  #	alica_engine/include/engine/RunningPlan.h
  #	alica_engine/include/engine/teammanager/TeamManager.h
  #	alica_engine/src/engine/RunningPlan.cpp
  #	alica_engine/src/engine/TeamObserver.cpp
  #	alica_engine/src/engine/model/ForallAgents.cpp
* remove unused RoleUsage
* clean capabilityDefinitionSet
* clean planningProblem and destinationPath
* fix planwriter issues
* test improvement after fixing subvariable bug
* more errors mkay
* fixing missing return
* fix comment issue
* fix remaining compiler issues in tests
* Update from Kassel & minor format update (`#10 <https://github.com/rapyuta-robotics/alica/issues/10>`_)
  * - test
  * Delete LICENSE
  For Hendrik ;-)
  * - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
  * Simplify the query interface
  * Fix test_alica_problem_composition
  * fix task_assignment test
  * remove debug flag
  * fix wrong filter in synctalk
  * Fix to fast path event + test for success from behavior
  * fix copy&paste issue
  * added .clang_format file
  * Update test_alica_authority.cpp
  That is more obvious.
  * - removed merge artifacts
  * copied stuff from old alica_test to alica_tests
  * added clang format file
  * moved coding guidelines into style folder
  * Update CodingGuidelines.md
  Merging the carpe-noctem-cassel coding guide with the one here in alica
  * Update CodingGuidelines.md
  update version
  * Update CodingGuidelines.md
  moved general things upwards + changes namespace examples
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * Update CodingGuidelines.md
  * - tryed to fix wrong naming for alica_test(s)
  - removed old test package
  * - fixed nameing
  * Update CodingGuidelines.md
  - moved pragma once into header section
  - added remark to relative includes
  * Update CodingGuidelines.md
  * style update
  * remove accidental log file
  * clean up
  * better whitespaces
  * fix bad merge
* update to compile supplementary and tests
* make alica engine a protected member in BasicBehaviour (`#9 <https://github.com/rapyuta-robotics/alica/issues/9>`_)
* Merge branch 'mergein-kassel' into hs_constify_model
  # Conflicts:
  #	alica_engine/include/engine/RunningPlan.h
  #	alica_engine/include/engine/teammanager/TeamManager.h
  #	alica_engine/src/engine/RunningPlan.cpp
  #	alica_engine/src/engine/TeamObserver.cpp
  #	alica_engine/src/engine/model/ForallAgents.cpp
* format update
* better whitespaces
* clean up
* style update
* Merge remote-tracking branch 'cnc-cassel/robotid' into mergein-kassel
  # Conflicts:
  #	alica_engine/include/engine/PlanBase.h
  #	alica_engine/include/engine/constraintmodul/ConditionStore.h
  #	alica_engine/include/engine/constraintmodul/Query.h
  #	alica_engine/src/engine/AlicaEngine.cpp
  #	alica_engine/src/engine/IRoleAssignment.cpp
  #	alica_engine/src/engine/PlanBase.cpp
  #	alica_engine/src/engine/StaticRoleAssignment.cpp
  #	alica_engine/src/engine/constraintmodul/ConditionStore.cpp
  #	alica_engine/src/engine/constraintmodul/Query.cpp
  #	alica_engine/src/engine/syncmodule/SyncModule.cpp
  #	alica_test/src/test/test_alica_gsolver_plan.cpp
  #	alica_test/src/test/test_alica_problem_composition.cpp
  #	alica_test/src/test/test_alica_sync_transition.cpp
  #	alica_tests/autogenerated/include/Plans/GSolver/SolverTestBehaviour.h
  #	alica_tests/autogenerated/include/TestWorldModel.h
  #	alica_tests/autogenerated/src/BehaviourCreator.cpp
  #	alica_tests/autogenerated/src/ConditionCreator.cpp
  #	alica_tests/autogenerated/src/ConstraintCreator.cpp
  #	alica_tests/autogenerated/src/Plans/Behaviour/ConstraintUsingBehaviour.cpp
  #	alica_tests/autogenerated/src/Plans/GSolver/SolverTestBehaviour.cpp
  #	alica_tests/autogenerated/src/Plans/ProblemModule/QueryBehaviour1.cpp
  #	alica_tests/autogenerated/src/Plans/ProblemModule/constraints/QueryPlan11479556074049Constraints.cpp
  #	alica_tests/autogenerated/src/UtilityFunctionCreator.cpp
  #	alica_tests/src/test/test_alica_authority.cpp
  #	alica_tests/src/test/test_alica_engine_behavior_pool_init.cpp
  #	alica_tests/src/test/test_alica_ros_proxy.cpp
  #	alica_tests/src/test/test_alica_simple_plan.cpp
  #	alica_tests/src/test/test_task_assignment.cpp
* Merge remote-tracking branch 'origin/rr-devel' into hs_constify_model
* Merge remote-tracking branch 'origin/rr-devel' into hs_constify_model
  # Conflicts:
  #	alica_engine/include/engine/Assignment.h
  #	alica_engine/include/engine/RunningPlan.h
  #	alica_engine/include/engine/constraintmodul/ConditionStore.h
  #	alica_engine/include/engine/model/ForallAgents.h
  #	alica_engine/include/engine/model/Quantifier.h
  #	alica_engine/include/engine/parser/ModelFactory.h
  #	alica_engine/include/engine/teammanager/TeamManager.h
  #	alica_engine/src/engine/Assignment.cpp
  #	alica_engine/src/engine/IRoleAssignment.cpp
  #	alica_engine/src/engine/RuleBook.cpp
  #	alica_engine/src/engine/RunningPlan.cpp
  #	alica_engine/src/engine/StaticRoleAssignment.cpp
  #	alica_engine/src/engine/UtilityFunction.cpp
  #	alica_engine/src/engine/allocationauthority/EntryPointRobotPair.cpp
  #	alica_engine/src/engine/collections/StateCollection.cpp
  #	alica_engine/src/engine/constraintmodul/Query.cpp
  #	alica_engine/src/engine/model/AbstractPlan.cpp
  #	alica_engine/src/engine/model/Behaviour.cpp
  #	alica_engine/src/engine/model/BehaviourConfiguration.cpp
  #	alica_engine/src/engine/model/ForallAgents.cpp
  #	alica_engine/src/engine/model/Plan.cpp
  #	alica_engine/src/engine/model/Quantifier.cpp
  #	alica_engine/src/engine/model/TerminalState.cpp
  #	alica_engine/src/engine/model/Variable.cpp
  #	alica_engine/src/engine/parser/ModelFactory.cpp
* model constification -> engine compiles
* more refactoring
* iteration 2
* Merge branch 'master' into robotid
* fix shared_ptr initialization with make_shared to prevent writing to null_ptr (`#7 <https://github.com/rapyuta-robotics/alica/issues/7>`_)
* Compiler error fixes, simplify ignoring agents (`#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_)
  * Compiler error fixes, simplify ignoring agents
  * remove vscode and add to gitignore
* Merge branch 'rapyuta-robotics-hs_fix_unit_tests' into robotid
* Merge branch 'hs_fix_unit_tests' of https://github.com/rapyuta-robotics/alica into rapyuta-robotics-hs_fix_unit_tests
* initial check-in, WIP
* Some improvements (`#6 <https://github.com/rapyuta-robotics/alica/issues/6>`_)
  * clean up
  * formatting changes
  * first simplification iteration on RunningPlan
* Alica engine static abort (`#5 <https://github.com/rapyuta-robotics/alica/issues/5>`_)
  * change alica engine abort to static
  * remove alica engine from IRoleAssignment
  * remove alica engine from Quantifier
  * removed alica engine from PlanParser constructor
  * address PR comments
* first simplification iteration on RunningPlan
* formatting changes
* clean up
* Merge pull request `#3 <https://github.com/rapyuta-robotics/alica/issues/3>`_ from rapyuta-robotics/clang5_formatter
  Clang5 formatter and applied formating
* clang5 formatter changes
* Various fixes (`#2 <https://github.com/rapyuta-robotics/alica/issues/2>`_)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
  * Simplify the query interface
  * Fix test_alica_problem_composition
  * fix task_assignment test
  * remove debug flag
  * fix wrong filter in synctalk
  * Fix to fast path event + test for success from behavior
  * fix copy&paste issue
* Merge branch 'rr-devel' into hs_fix_unit_tests
* Fix to fast path event + test for success from behavior
* fix wrong filter in synctalk
* remove debug flag
* Fix Authority unit test and multi-agent unittest (`#1 <https://github.com/rapyuta-robotics/alica/issues/1>`_)
  * style update Planbase, add acessor to find out if planbase is waiting for steppong signal
  * factor out unit test main
  * fix wrong compare
  * Fix authority system and unit test
  * avoid unnecessary dereferencing of entypoints
  * address review comments
  * Fix local agent wrongly indicating a team updat
  * log robotids, not their adress
  * remove unneeded accessor
  * add some constness to agent
  * improve robotID debug output
  * Do not clear success marks of the local agent all the time.
  * prevent doubling of success marks...
  * minor improvements
  * change tests to rostests, dependency is there anyway.
  * address review comments
  * add rostest dependency
  * remove long time limit again
* fix task_assignment test
* Fix test_alica_problem_composition
* Simplify the query interface
* address review comments
* minor improvements
* prevent doubling of success marks...
* Do not clear success marks of the local agent all the time.
* improve robotID debug output
* add some constness to agent
* remove unneeded accessor
* log robotids, not their adress
* Fix local agent wrongly indicating a team updat
* address review comments
* avoid unnecessary dereferencing of entypoints
* Fix authority system and unit test
* fix wrong compare
* style update Planbase, add acessor to find out if planbase is waiting for steppong signal
* - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
* - fix c++ style
* disable debug outpur
* - modified a lot of debug output
  - refactored for better code style
* - fixed code style
* - adapted to new supplementary subfolder
* - improved output
* - changed availTime type to AlicaTime
* Merge pull request `#3 <https://github.com/rapyuta-robotics/alica/issues/3>`_ from carpe-noctem-cassel/StephanOpfer-patch-1
  Delete LICENSE
* Delete LICENSE
  For Hendrik ;-)
* - refactored to general agentid
* changed ae constructor
* fix teammanager bug
* couts
* - some fixes
* - refactor sync module
* refactor some interface bullshit
* changed hacsh and equals in unordered set
  replaced factory by manager
* - current status
* hmpf
* - introduced the agentIDmanager
* wrong iterator
* added missing iterator assignment after erase on list
* fixed id comparison
  fixed find
  fixed map
  fixed default uitlity assignment
  set local agent to active
* trying to fix successmarks
  left couts in for further debugging
* fixed wrong calculation from vec uint8 to int
* bug fixes
* fix
* add  teammanager init call
* added const
* alica test refactoring and bug fixing
* - refactored from Robot ID to Agent ID
* - refactored robot_id into agent_id in supplementary
* refactoring robotid
* ros_proxy compiles again
* - first compiling commit
* - further adaption
* - further adaption to new IRobotID and TeamManager
* - further adaption to new IRobotID and TeamManager
* - initial commit for team manager
* - started to fix ros alica proxy
  - todo: create methods for creating id stuff
* - refactored alica from (int, short, long)-IDs to IRobotID
  - deleted unused old role assignment
* something is compiling omg
* - initila commit for robotid
* - fixing reserve bug (wrong vector size)
* comments
* added ifdef to couts
* - fixed from comparision
* - first try to overload <<operator for UniqueVariableStore
* - current state of refactoring
  - current problem is why the probParts are not properly initialized
* - added nullptr check in getSolution of Query
  - added solver into composition test
  -> next fix segfault because of non existing variables in problem part
* - let the static role assignment also inform the team observer
  - fixed the model factors (now it also parses variables and parametrisations of plan types)
  - fixed task assignment test (actually this is not testing, except executing some code)
* update in progress
* - auto generated plans for problem building test
  - improved plan writer test (it now deletes it written files when the test was successfull, because the written files made the plan designer autogenerate cpp stubs two times for the same plan)
* - cleaned some code of RoleAssignment
  - implemented StaticRoleAssignment
  - fixed bugs in alica_test (now tests run again)
* fixed bug at removeConstraint
* - refactored the rest of getSolution method in Query
* - polished the hasVariable method of the ProblemPart
* - further refactoring the acceptQuery method of the ConditionStore class
* - refactored a lot of names
  - found bug in conditionStare::removeCondition (never removed a condition)
* dont know
* - Removed Logger-Class and put everything into a namespace
  - rearranged parameters
* added comment
* Revert "some warnings fixed"
  This reverts commit ee9d141ea90398db812e1f9ca712333fb6eb398b.
* some warnings fixed
* removed warnings of new cmake version
* do some more forward declaration
* - removed old default comments
* fixed some declarations
* - deleted three methods from running plan
  - created a new anyChildrenTaskSuccess
  - repaired alica logger
* - removed output about unknown state
* removed output
* small bugfix
* bugfix
* rafactoring in some internal stuff
* small refactoring
* multiple small bugfixes
* bugfix
* cachData bug and missing lock guard fixed
* removed todo
* added log messages
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* fixed hardest bug ever!
* added "enabled" to to_string method
* comments
* alica fixes
* bug fix
* minor clean up in team observer - hope it does not break something
* taker fix to get simulator working
* removed commented stuff
* fixed one bug recursive bug in PartialAssignement and one in getTaskId
* expanded basic beh
* fix some shard_ptr stuff
* fixed some buggs in alica engine
* ae fixes
* fixed alica tests, alica engine segfault
* elaborated todos
* -removed unnecessary project
  -changed ConstraintQuery-Constructor Parameter (AlicaEngine instead of Behaviour)
* update cmakelist.txt and removed rqt
* fixes in Constraint Solver
* fixed variables
* removed inheritance solverterm -> solvervariable
* renamed function
* added philipps full name
* fix role assignment for capabilities with just one capability value
* show behaviour name in case of exception
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* Fixes Test
* return to ""
* expanded basic beh
* added getHicherEntrypoint to basicBehaviour
* SuccesMark fix
* Fix the leak and a few test fixes
* added getParentEntryPoint
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* renamed BehaviourEngineInfo into AlicaEngineInfo
* fixed dependencies
* refactored typedef for time
* fix output
* fixes
* some fix for trigger
* removed warnings
* bla
* removed cout
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* added trigger test
* added condition timeout
* fixed not parsing same file twice
* inserted a more clear comment
* log changes
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* bug fixing
* catching exceptions in behaviours and conditions now
* bla
* removed test
* testflag
* segfault up `#1 <https://github.com/rapyuta-robotics/alica/issues/1>`_
* Fix planWriter test
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* Sync-Transition Test
* fixed parsing of parameters
* removed some autogenerated todos
* basic behavior changes
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* changed buildchain for audicup
* VarSyncModule - removed double
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* solver updates
* cleanup and bugfux
* updates for solver
* dummysolver works... CGSolver ready for testing
* ...
* ...
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/src/engine/AlicaEngine.cpp
* resultstore
* - fixed authority test
  - removed debug output
* fixed some bugs
* fixed sorting error ... preduced anotherone
* changed cmake to be catkin-independent
* some more fixes, and searching for authority bug
* fixed horrible bug... static vs nonstatic get<bool>-systemconfig lets you change your config file
* Merge branch 'master' into assignment_collection_fix
* some debug
* reworked AssignmentCollection
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* added constraint test
* added todo
* bug fixes
* done know anymore
* working on authritymanager
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_test/autogenerated/src/Plans/Authority/AuthorityTest1414403413451.cpp
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_test/autogenerated/src/Plans/Authority/AuthorityTest1414403413451.cpp
* working on tests
* private desctructor?
* fixed stuff
* working on tests
* serialization stuff + fixes
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* finished planwriter test
* working on planwriter
* working on planwriter test
* bug fixes
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* debugging...
* ...
* updates
* alica doesnt depend from autodiff anymore
* heavy bug resolved
* strange behaviour
* bug fixes
* bug fixes
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* bug fixes
* ...
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/src/engine/model/ForallAgents.cpp
* bug fixes
* bug fixes
* improved test ... revealed new errors
* fix successmarks
* bug fixes
* working on multiagent test
* removed static  partial assignments
* see previous commit
* removed static stuff
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* removed singleton
* fixed not required singleton access
* stuff
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* ..
* began adding solver stuff to alica
* fixed robot dupplication at serveral places and revealed a bug seee test
* improved test and made fixes test
* condintions are working now and we have a test for it
* fixes
* fixed memory leaks
* sodele
* fixes for memory leaks
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* fixed timing bug
* replaced unsigned long with alicaTime
* memory leaks and alica clock fixed
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* changed
* code examples for codegeration
* fixes
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* bugfixes
* working on bugs
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* working on bugs
* case
* same
* removed singelton calls
* bug fixes
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* sodele
* added communication
* remove workingset
* added doxygen comments
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
  Conflicts:
  alica_plan_designer/bin/linux.gtk.x86_64/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workingsets.xml
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* added doxygen comments
* Ros communication proxy is done
* working on doxygen comments
* added doxygen comments
* bug fixing removed some unnessasary cout
* bug fixes
* bug fixing
* working onauthoritymanger
* finished planwriter
* implementation
* comment
* fixes
* cleaned up communication interface
* working on planwriter
* bug fixes
* bug fixes
* bug fixes
* bug fixes
* Bug Fiiiix
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* change return for collections to &
* changed returns of collections to reference
* Bug Fix, Bug fix, return collections only with &
* Finish SyncModul
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* fix
* finished synchronisation
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* SynchModul
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* working on synchronisation
* SynchModul
* working on synchronisation
* SyncModul
* fix
* rename folders and create syncmodul package
* bug fixing
* Fix bugs and change Tests
* working on bug
* reorganiced tests
* initializing every missing paramerter in constructors finished runningplan
* bug fix
* BUG FIXES
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/src/engine/RunningPlan.cpp
* Fix
* some changes
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* some fixes
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/src/engine/logging/Logger.cpp
  alica_ros_proxy/src/clock/AlicaROSClock.cpp
* fix
* bug fix
* some fix
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
  Conflicts:
  alica_ros_proxy/src/clock/AlicaROSClock.cpp
* try to fix some stuff
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/src/engine/logging/Logger.cpp
  alica_ros_proxy/src/clock/AlicaROSClock.cpp
* working on logger
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* removed nicknames
* fixed some bugs
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/src/engine/AlicaEngine.cpp
  alica_engine/src/engine/collections/RobotEngineData.cpp
  alica_engine/src/engine/teamobserver/TeamObserver.cpp
  alica_plan_designer/bin/linux.gtk.x86_64/workspace/.metadata/.plugins/org.eclipse.core.resources/.projects/ExpressionValidators/.location
  alica_plan_designer/bin/linux.gtk.x86_64/workspace/.metadata/.plugins/org.eclipse.core.resources/.projects/Misc/.location
  alica_plan_designer/bin/linux.gtk.x86_64/workspace/.metadata/.plugins/org.eclipse.core.resources/.projects/Plans/.location
  alica_plan_designer/bin/linux.gtk.x86_64/workspace/.metadata/.plugins/org.eclipse.core.resources/.projects/Roles/.location
  alica_plan_designer/bin/linux.gtk.x86_64/workspace/.metadata/.plugins/org.eclipse.ui.workbench/workingsets.xml
* fixed some bugs
* Bug fixes
* fixed bug in cycle manaer addded defaultutility started taskassignment test
* finished cyclemanager
* working on cyclemanager
* finished logger
* working on logger
* added alica time
* finished allocationDifference
* Test now fine
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* BugFix
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* working on runningplan added alocationdifference and entryPointRobotPair
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/src/engine/RunningPlan.cpp
* PlanBase done
* added several methods to runningplan
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* working on planselector
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/include/engine/IPlanSelector.h
  alica_engine/include/engine/RunningPlan.h
  alica_engine/include/engine/planselector/PlanSelector.h
  alica_engine/src/engine/RunningPlan.cpp
* added functions to ruleBook, runningPlan
* working on planselector
* working onassignment
* working on assignment
* finished utilityFunction
* working on utilityfunction
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* Added functions to RuleBook and Assignment
* finished USummand
* removed USummand.cpp
* Added Stuff to rulebook and assignment
* added usummand and utilityinterval
* added hash templates
* added taskRoleStruct
* working on taskassignment
* added IPlanSelector
* Added new stuff
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/include/engine/AlicaEngine.h
* Changes
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* some line
* working on taskassignment
* working on partialAssignment
* working on partial assignment
* working on task assignment
* added IAlicaCommunication to AlicaEngine
* working on roleassignment
* fixed test
* fix
* Bug Fix
* PlanParser PlanningProblem
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* added PlanningProblem to planParser
* working on roleassignment
* some more stuff for the behaviour pool
* fix merge
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
  alica_engine/include/engine/Assignment.h
  alica_engine/include/engine/IAssignment.h
  alica_engine/src/engine/RunningPlan.cpp
* PlanBase
* trying to fix test
* added autodiff stuff to quantifier variable and forallagents
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
  Conflicts:
* moved IAlicaComm to inlcude
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* moved
* adding autodiff to model
* finished collections and created assingment and iassignment
* some fix
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* working on collection
* working on collections
* - introduced more features into the test plans for the plan parser test
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* FIX BUG
* working on collections
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* fix
* fix the bug of last commit ;-)
* Merge branch 'master' of barbuda.vs.eecs.uni-kassel.de:~/git-repos/alica
* probably a bug
* fixed segmentation faul
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* Merge branch 'master' of ssh://barbuda.vs.eecs.uni-kassel.de/~/git-repos/alica
* mved stuff
* Contributors: Abhishek S, Abhishek Sharma, AbhishekS, Alex, Alexander Bolinsky, Brain2, CNBasement, Carpe Noctem, Christoph Eickhoff, Dale Koenig, David Simões, Dzmitry Ivashniou, Endy, Gautham Manoharan, Hendrik, Hendrik Skubch, José Mendes Filho, Karasuma1412, Luca Tricerri, Maksim Derbasov, Paul Panin, Philipp, Renan Salles, Rogerio Fonteles, Stefan Jakob, Stefan Niemczyk, StefanJakob, StefanSchmelz, Stephan Opfer, Thomas Falk, Tobias, Veeraj S Khokale, Witali Schmidt, athish-t, bjoernschroeder, bschroeder, christianhelm, cyberdrk, dhananjay-patki, gajen, lab-pc5, lab-pc6, mansiVerma26, nase, ppa, veerajsk, william
