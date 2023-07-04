^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package supplementary_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
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
* Remove legacy transitions (`#455 <https://github.com/rapyuta-robotics/alica/issues/455>`_)
  remove LegacyTransitionConditions
* reexport plans for alica_tests and supplementary (`#457 <https://github.com/rapyuta-robotics/alica/issues/457>`_)
  * reexport plans for alica_tests and supplementary
  * add lib name to runtime condition
  * format
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
* Remove codegen from supplementary_tests (`#419 <https://github.com/rapyuta-robotics/alica/issues/419>`_)
  * move behaviours into libalica-tests, use dynamic loading
  * add dynamic loading for TestBehaviour
  * remove libalica-tests from dependencies
  * use dynamic loading for behaviours in supplementary_tests
  * remove old line, fix typo
  * set Rapyuta Robotics as author / maintainer for libs
  * remove package files for libs
  * move add_subdirectory out of if block
  * use dynamic loading for all alica elements in supplementary_tests
  * remove preConditionConstraints, move VariableHandlingStart transition condition to lib, fix dynamic loading test
  * format
  * - remove unused includes
  - use standard library for conditions
  - replace extern bool vhStartCondition with globalBlackboard
  - remove empty destructors of alica elements
  * remove cmake files for libaries, remove unused condition file in supplementary_tests
  * remove expr folder from supplementary_tests
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Use dynamic loading with behaviours (`#410 <https://github.com/rapyuta-robotics/alica/issues/410>`_)
  * move behaviours into libalica-tests, use dynamic loading
  * add dynamic loading for TestBehaviour
  * remove libalica-tests from dependencies
  * use dynamic loading for behaviours in supplementary_tests
  * remove old line, fix typo
  * set Rapyuta Robotics as author / maintainer for libs
  * remove package files for libs
  * move add_subdirectory out of if block
  * Use standard library
  * combine cmakelists
  * Remove extra 'lib'
  ---------
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Eliminate codegen from behaviors in supplementary tests (`#403 <https://github.com/rapyuta-robotics/alica/issues/403>`_)
  * step 001 remove TAGS
  * step 002
  ---------
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
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
* Fix dynamic loading in supplementary (`#369 <https://github.com/rapyuta-robotics/alica/issues/369>`_)
  * Fix dynamic loading in supplementary
  - Remove pml, beh & cnd files of the dynamically loaded lib from the
  root etc folder so that code is not generated for it
  - Fix the dynamically loaded lib's pml, beh & cnd files. Generate them
  using the alica designer
  - Remove the extra cnd file for acme runtime condition
  - Export symbols using the correct names
  - Remove autogenerated code for the removed pml, beh & cnd files
  * Fix format
* Fix code generation for conditions (`#367 <https://github.com/rapyuta-robotics/alica/issues/367>`_)
  * Fix codegen, fix tests
  * Fix supplementary tests
  * Fix ros1 turtlesim
  * Fix ros2 turtlesim
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
* Moved all Ros1 dependent packages to a subfolder (`#348 <https://github.com/rapyuta-robotics/alica/issues/348>`_)
* Contributors: Dale Koenig, Luca Tricerri, Maksim Derbasov, bjoernschroeder, dhananjay-patki, veerajsk
