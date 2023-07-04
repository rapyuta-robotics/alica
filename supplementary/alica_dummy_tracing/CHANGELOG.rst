^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_dummy_tracing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Update implementation of DummyAlicaTracing (`#383 <https://github.com/rapyuta-robotics/alica/issues/383>`_)
  * update implementation of dummyAlicaTracing to use latest IAlicaTracing interface
  * format
  * dont override setLog
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
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
* Created alica dummy tracing to use in lbc_tests (`#228 <https://github.com/rapyuta-robotics/alica/issues/228>`_)
  * created dummy alica tracing
  * changed dir structure
  * removed unnecessary headers
  * removed extra lines
* Contributors: Dale Koenig, Luca Tricerri, bjoernschroeder, dhananjay-patki, mansiVerma26
