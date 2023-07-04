^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_simple_solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Remove engine references from constraintsolver (`#398 <https://github.com/rapyuta-robotics/alica/issues/398>`_)
  * Remove engine ref from ISolver
  * Fix supplementary tests
  * Fix turtlesims
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
* Run precommit on all files (`#340 <https://github.com/rapyuta-robotics/alica/issues/340>`_)
* Removed Redundant cmake Libraries (`#336 <https://github.com/rapyuta-robotics/alica/issues/336>`_)
* Removed ROS1 and Boost dependencies (`#337 <https://github.com/rapyuta-robotics/alica/issues/337>`_)
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
* Remove/Deprecate blackboard edit functions (`#245 <https://github.com/rapyuta-robotics/alica/issues/245>`_)
* Change cmake version (`#170 <https://github.com/rapyuta-robotics/alica/issues/170>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
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
* Merge pull request `#106 <https://github.com/rapyuta-robotics/alica/issues/106>`_ from rapyuta-robotics/json-plan-format
  Json plan format
* Merge pull request `#103 <https://github.com/rapyuta-robotics/alica/issues/103>`_ from rapyuta-robotics/repair-tests
  Repair tests
* Merge pull request `#107 <https://github.com/rapyuta-robotics/alica/issues/107>`_ from rapyuta-robotics/featureCommonConfs
  Feature common confs
* Merge branch 'v0.9.0' into featureCommonConfs
* Merge pull request `#105 <https://github.com/rapyuta-robotics/alica/issues/105>`_ from rapyuta-robotics/dk_install_fixes
  CMake install fixes
* Install headers
* CMake install fixes
* Merge branch 'rapyuta-robotics-v0.9.0' into integrate-auto-discovery
* Merge branch 'v0.9.0' of https://github.com/rapyuta-robotics/alica into rapyuta-robotics-v0.9.0
* Merge pull request `#14 <https://github.com/rapyuta-robotics/alica/issues/14>`_ from rapyuta-robotics/rr_to_daisys
  Refactoring into 3 repo
* Merge branch 'new_api' of github.com:rapyuta-robotics/alica into new_api
* Revert "move to c++14, clean cmakelist files"
  This reverts commit 4ee14c88f6e460921f704980f718dd56265ecde3.
* Merge branch 'rr-devel' into new_api
* Switch to C++14 (`#59 <https://github.com/rapyuta-robotics/alica/issues/59>`_)
  * move to c++14, clean cmakelist files
  * fix eclipse cpp version
* move to c++14, clean cmakelist files
* Merge pull request `#48 <https://github.com/rapyuta-robotics/alica/issues/48>`_ from rapyuta-robotics/new_alica
  WIP to move to 3 repo structure
* Further sync with new supplementary and essentials
* Merge branch 'newPD' of github.com:carpe-noctem-cassel/alica into newPD
* Merge branch 'master' into newPD
* Contributors: Abhishek S, AbhishekS, Dale Koenig, Hendrik, Hendrik Skubch, Luca Tricerri, Maksim Derbasov, Stephan Opfer, Witali Schmidt, bjoernschroeder, christianhelm, dhananjay-patki, jironitta
