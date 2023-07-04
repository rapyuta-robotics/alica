^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_dynamic_loading
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Fixup adjacent success test using the new template [1] (`#490 <https://github.com/rapyuta-robotics/alica/issues/490>`_)
  Fixup adjacent success test using the new template
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
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
* Add alica_standard_library package to use for common conditions (`#411 <https://github.com/rapyuta-robotics/alica/issues/411>`_)
* Log improvements (`#405 <https://github.com/rapyuta-robotics/alica/issues/405>`_)
  * Log improvements
  * Improve
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
* Fix code generation for conditions (`#367 <https://github.com/rapyuta-robotics/alica/issues/367>`_)
  * Fix codegen, fix tests
  * Fix supplementary tests
  * Fix ros1 turtlesim
  * Fix ros2 turtlesim
* Fix path problem on dynamicloading and githubaction (`#347 <https://github.com/rapyuta-robotics/alica/issues/347>`_)
  Fix pathfinding in test
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
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
* Contributors: Dale Koenig, Luca Tricerri, Maksim Derbasov, bjoernschroeder, veerajsk
