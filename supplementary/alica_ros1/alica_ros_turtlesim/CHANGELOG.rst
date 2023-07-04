^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_ros_turtlesim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
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
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Additional tutorials for turtlesim (`#462 <https://github.com/rapyuta-robotics/alica/issues/462>`_)
  * Dale-Koenig changes 09:12:41.488362
  * Fixes and beh implementation
  ---------
  Co-authored-by: Dale-Koenig <default@default.com>
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
* Add integration test for turtlesim (`#443 <https://github.com/rapyuta-robotics/alica/issues/443>`_)
  * draft of alica_ros_turtlesim tests
  * changed test
  * test CI
  * fixed tests with less macros and more precise robot stopping
  * fixed compilation
  * corrected tests
  * changed all tolerances to 0.1
  * addressed Dale's comments
  * increased tolerance a little bit because some edge cases are failing by 0.02, changed dockerfile
  * changed : to .
  * added const to const vars
  ---------
  Co-authored-by: Rogerio Fonteles <rogerio.fonteles@moley.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Fix roleset name to match rst file name (`#452 <https://github.com/rapyuta-robotics/alica/issues/452>`_)
* Refactor turtlesim and add additional simpler tutorial plan (`#432 <https://github.com/rapyuta-robotics/alica/issues/432>`_)
  * Plans to refactor turtlesim and add new tutorial
  * Save turtle refactor
  * More changes
  * Fix
  * Update for new tutorial
  * new files
  * Remove master.pml
  * Dale-Koenig changes 08:05:02.929044
  * Semi working
  * Extra log
  * Split double generation to avoid bug
  * Fix utl function name
  * Remove empty file
  * Fix tests
  * Rename turtle to turtle interfaces
  * Refactor turtle interfaces usage
  * Fixes
  * Fix build
  * Use BasicPlan
  * Regenerate etc with updated designer
  ---------
  Co-authored-by: Dale-Koenig <default@default.com>
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
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
* Cleanup (`#426 <https://github.com/rapyuta-robotics/alica/issues/426>`_)
* Removing defaultValue (`#422 <https://github.com/rapyuta-robotics/alica/issues/422>`_)
* alica_ros_turtlesim fixed for more than (core number * 2) robots (`#417 <https://github.com/rapyuta-robotics/alica/issues/417>`_)
  turtlesim alica fixed for more than (core number * 2) robots
  Co-authored-by: Rogerio Fonteles <rogerio.fonteles@moley.com>
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Fix readme references to stand condition library (`#420 <https://github.com/rapyuta-robotics/alica/issues/420>`_)
  Fix readme
* Add alica_standard_library package to use for common conditions (`#411 <https://github.com/rapyuta-robotics/alica/issues/411>`_)
* Add simulation plan details to turtlesim readme (`#401 <https://github.com/rapyuta-robotics/alica/issues/401>`_)
  * Add simulation plan details to turtlesim readme
  * Update supplementary/alica_ros1/alica_ros_turtlesim/README.md
  * Precommit
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Fix simulation blackboard key access (`#399 <https://github.com/rapyuta-robotics/alica/issues/399>`_)
* Remove engine references from constraintsolver (`#398 <https://github.com/rapyuta-robotics/alica/issues/398>`_)
  * Remove engine ref from ISolver
  * Fix supplementary tests
  * Fix turtlesims
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
* Fix code generation for conditions (`#367 <https://github.com/rapyuta-robotics/alica/issues/367>`_)
  * Fix codegen, fix tests
  * Fix supplementary tests
  * Fix ros1 turtlesim
  * Fix ros2 turtlesim
* Minor cleanup (`#355 <https://github.com/rapyuta-robotics/alica/issues/355>`_)
  * Minor cleanup
  * Fixes
  * Fixes
* Moved all Ros1 dependent packages to a subfolder (`#348 <https://github.com/rapyuta-robotics/alica/issues/348>`_)
* Contributors: Dale Koenig, Luca Tricerri, Maksim Derbasov, Rogerio Fonteles, bjoernschroeder, dhananjay-patki, veerajsk
