^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_standard_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fixup adjacent success test using the new template [1] (`#490 <https://github.com/rapyuta-robotics/alica/issues/490>`_)
  Fixup adjacent success test using the new template
* Comparator conditions in alica_standard_library  (`#478 <https://github.com/rapyuta-robotics/alica/issues/478>`_)
  * initial draft
  * add lessthan, greaterthan, notequal
  * add some tests
  * small change
  * review comments
  * add tests
* Add IsChildFailure to alica standard library (`#481 <https://github.com/rapyuta-robotics/alica/issues/481>`_)
  add ischildfailure
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Add IsChildSuccess condition (`#463 <https://github.com/rapyuta-robotics/alica/issues/463>`_)
  * add IsChildSuccess condition
  * add test
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
* Get rid of code gen in alica tests (`#440 <https://github.com/rapyuta-robotics/alica/issues/440>`_)
  * set lib name
  * Export Symbols for Plans and UtilityFunctions (`#441 <https://github.com/rapyuta-robotics/alica/issues/441>`_)
  * export symbols
  * Dynamic loading for conditions (`#442 <https://github.com/rapyuta-robotics/alica/issues/442>`_)
  * add dynamic loading for conditions
  * fix condition name, add implementation for IsAnyChildStatusSuccess
  * move conditions out of standard library, rename default condition to AlwaysFalseCondition
  * Dynamic loading for plans + utility functions (`#444 <https://github.com/rapyuta-robotics/alica/issues/444>`_)
  * dynamic loading for plans and utility functions
  * Add dynamic loading for conditions and constraints (`#445 <https://github.com/rapyuta-robotics/alica/issues/445>`_)
  * add dynamic loading for conditions and constraints
  * fix plan parser test, activate tests
  * export constraints
  * try cmake fixes
  * Fix tests
  * cleanup preconditions
  ---------
  Co-authored-by: Veeraj S Khokale <veeraj.khokale@rapyuta-robotics.com>
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
* Add alica_standard_library package to use for common conditions (`#411 <https://github.com/rapyuta-robotics/alica/issues/411>`_)
* Contributors: Dale Koenig, Prajapati-Pawan, SaawanPate, bjoernschroeder, veerajsk
