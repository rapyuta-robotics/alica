^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Update scheduling tests to new design (`#499 <https://github.com/rapyuta-robotics/alica/issues/499>`_)
  * export plans
  * update tests
  * implement tests
  * update tests
  * remove unused plans
  * use STEP_UNTIL_ASSERT_TRUE
  * review comments
  * address review comments
  * remove unused conditions, fix ExecOrderTestPlan
  * check that behaviourBAA run is called in order
  * add additional steps to repeated run test
  * add output on test failure for CI
  * simplify check for repeated run calls
  * remove log, only one additional STEP_UNTIL
  * increase number of steps
  * log callCounter and expected runs
  * log times
  * initialize counter
  * remove logs
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
* Replace world model test with global bb test (`#495 <https://github.com/rapyuta-robotics/alica/issues/495>`_)
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
* Comparator conditions in alica_standard_library  (`#478 <https://github.com/rapyuta-robotics/alica/issues/478>`_)
  * initial draft
  * add lessthan, greaterthan, notequal
  * add some tests
  * small change
  * review comments
  * add tests
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
* Move alica test files (`#474 <https://github.com/rapyuta-robotics/alica/issues/474>`_)
  * Move test files
  * Fix
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
* Minor cleanup (`#466 <https://github.com/rapyuta-robotics/alica/issues/466>`_)
  * remove print, catch exception in ValueMappingCondition, use locked blackboard for tests
  * Use UnlockedBlackboard for blackboard tests
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
* Remove legacy transitions (`#455 <https://github.com/rapyuta-robotics/alica/issues/455>`_)
  remove LegacyTransitionConditions
* reexport plans for alica_tests and supplementary (`#457 <https://github.com/rapyuta-robotics/alica/issues/457>`_)
  * reexport plans for alica_tests and supplementary
  * add lib name to runtime condition
  * format
* Remove domain elements (`#453 <https://github.com/rapyuta-robotics/alica/issues/453>`_)
  * remove DomainPlan
  * remove DomainBehaviour
  * remove DomainCondition
  * Move alica elements to alica_tests (`#454 <https://github.com/rapyuta-robotics/alica/issues/454>`_)
  * remove Expr folder, move everything to libalica-tests
  * remove expr folder from cmakelists
  * delete Expr folder after merge
  * move files to alica_tests
  * remove libalica-tests include in cmake file
* Transfered alica_tests from ros1 folder to alica folder (`#450 <https://github.com/rapyuta-robotics/alica/issues/450>`_)
  * transfered folder
  * transfered again to alica folder
  ---------
  Co-authored-by: Rogerio Fonteles <rogerio.fonteles@moley.com>
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
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
* Put all tests behind BUILD_TESTING cmake flag (`#346 <https://github.com/rapyuta-robotics/alica/issues/346>`_)
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
* Remove TestWorldModel singleton (`#326 <https://github.com/rapyuta-robotics/alica/issues/326>`_)
  * remove singleton TestWordModel
  * first step
  * fix tracing tests
  * fix tracing tests
  * fix tracing tests
  * fix format
  * refactoring
  * refactory
  * fix last check
  * fix PR request
  * minor
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
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
* Update conditions (`#307 <https://github.com/rapyuta-robotics/alica/issues/307>`_)
  * update alica_tests
  * regenerate files
  * update supplementary tests
  * update turtlesim
  Co-authored-by: bjoernschroeder <bjoernschroder@rapyuta-robotics.com>
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
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
* Creators fixes (`#264 <https://github.com/rapyuta-robotics/alica/issues/264>`_)
  * Creators fixes
  * Ensure stepEngine does not deadlock
  * Various fixes with creators
  * More fix
  * Revert mistaken change in tag
* Fix generate script and shorten constructors (`#263 <https://github.com/rapyuta-robotics/alica/issues/263>`_)
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
* Prevent generation of empty constraint files (`#257 <https://github.com/rapyuta-robotics/alica/issues/257>`_)
  * prevent generation of empty constraint files
  * update codegen
  * remove empty constraints from turtlesim
  * remove empty constraints from supplementary tests
  * remove empty constraints from alica_tests
  * increase alica_tests time limit
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* Change overlapping id's (`#256 <https://github.com/rapyuta-robotics/alica/issues/256>`_)
* Pass context in behaviour and plan creation and add codegeneration jar files (`#249 <https://github.com/rapyuta-robotics/alica/issues/249>`_)
  * Simplify construction of behaviour, new argument shouldn't require app code regeneration
  * Introduce PlanContext
  * Update alica tests
  * Update supplementary tests
  * Update alica test utility
  * Enable git lfs for jar files
  * Update readme
* Remove/Deprecate blackboard edit functions (`#245 <https://github.com/rapyuta-robotics/alica/issues/245>`_)
* Blackboard json (`#239 <https://github.com/rapyuta-robotics/alica/issues/239>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Multiple alica instances (`#241 <https://github.com/rapyuta-robotics/alica/issues/241>`_)
  * ignore agentAnnouncements of agents using different rolesets
  * store masterPlanId in planHash, fix planHash overflow, remove unnecessary check
  * use uint64 for planhashes in messages, store planhash in agentQuery as uint64_t, set planhashes in assignment tests
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* Test launch file improvements (`#240 <https://github.com/rapyuta-robotics/alica/issues/240>`_)
  These modificiations are already used in other projects
  Xterm config
  Gtest filter
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
* Creation on nullptr check for PlanContext (`#221 <https://github.com/rapyuta-robotics/alica/issues/221>`_)
  * add isValid method for nullptr check, add unittest
  * format
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* only query for results until one result has been found (`#220 <https://github.com/rapyuta-robotics/alica/issues/220>`_)
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
* Change cmake version (`#170 <https://github.com/rapyuta-robotics/alica/issues/170>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* get spinner reference from application (`#192 <https://github.com/rapyuta-robotics/alica/issues/192>`_)
  * get spinner reference from application
  update turtlesim tutorial code
  fix lifetime of spinner
  timer and communicator should receive spinner and cbq
  update tests
  fix spinner in test
  fix spinner in tests
  missed this test
  only pass cbq to timer and communicator
  start and stop spinner at appropriate places
  fix spinner starting order in tests
  fix
  fix spinners in tests
  fix
  update tests after merge
  * code format
  * fixes after merge
  * more fixes after merge
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
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
* regenerate, fix onInit / onTerminate / run overrides for plan headers (`#207 <https://github.com/rapyuta-robotics/alica/issues/207>`_)
  * regenerate, fix onInit / onTerminate / run overrides for plan headers
  * regenerate turtlesim and supplementary_tests
  * move methods to override into protected region pro
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
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
* add unittest for behaviour execution with subplans (`#166 <https://github.com/rapyuta-robotics/alica/issues/166>`_)
  * add unittest for behaviour execution in subplans
  * execute behaviour in parallel to plan
  * move files in Expr back to autogenerated
  * move files from autogenerated to Expr (`#168 <https://github.com/rapyuta-robotics/alica/issues/168>`_)
  * track execution of init/run/terminate with execOrder
  * remove WaitPlan
  * remove WaitPlan pml, regenerate
  * remove unused plans
  * simplify test
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
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
* Bas tracing test + TestTracing implementation (`#177 <https://github.com/rapyuta-robotics/alica/issues/177>`_)
  * implement TestTracing
  * add fixture for testTracing
  * create plan for testing, regenerate
  * fixes after rebase
  * use size_t for vector size comparison
  * regenerate files
  Co-authored-by: veerajsk <54059004+veerajsk@users.noreply.github.com>
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
* Merge pull request `#136 <https://github.com/rapyuta-robotics/alica/issues/136>`_ from rapyuta-robotics/feature/use_current_assignments
  Use Existing Assignments in Utility Summands
* - extended the alica engine's USummand interface to also use the old assignment
  - adapted all existing Summans in alica_tests
  - wrote a new test that uses the extended interface
* Merge branch 'rr-devel' into doxygen_integration
* Merge branch 'rr-devel' into gh-pages
* Merge pull request `#133 <https://github.com/rapyuta-robotics/alica/issues/133>`_ from rapyuta-robotics/fix_parsing_quantifiers
  replace "ALL" with "all"
* Merge branch 'rr-devel' into fix_parsing_quantifiers
* fix quantifiers in test plans
* Merge pull request `#131 <https://github.com/rapyuta-robotics/alica/issues/131>`_ from rapyuta-robotics/improve_comment_on_id
  Improve Doxygen Comment
* - removed unused deprecated config
  - improved comment on id parameter in AlicaContextParams
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
* adapt to new id api
* Merge branch 'rr-devel' into revise_alica_test_utility_api
* Merge pull request `#116 <https://github.com/rapyuta-robotics/alica/issues/116>`_ from rapyuta-robotics/remove_id_api_from_context
  removed forwarding id API
* - moved isStateActive API to test UTIL
* introduced Util class for separating TestContext from Alica internal tests
* - cleanup alica_tests with improved TestContext-API
* - improve launch file, by adding gdb support
  - fix behaviour trigger test
* first step to run tests without getBehaviourPool()
* removed forwarding id API
* Merge pull request `#114 <https://github.com/rapyuta-robotics/alica/issues/114>`_ from rapyuta-robotics/json-plan-format
  Wrong merge...
* Merge pull request `#112 <https://github.com/rapyuta-robotics/alica/issues/112>`_ from rapyuta-robotics/sop_test_framework
  Initial Version of the Alica Test Utilities
* Merge pull request `#106 <https://github.com/rapyuta-robotics/alica/issues/106>`_ from rapyuta-robotics/json-plan-format
  Json plan format
* fix comporison of unsigned and signed int
* - updated tests to run without engine getter
  - introduced extra getDomainVariable method in TeamManager
* adapted alica to the improved event_handling package
* - fix warnings
* fix warning
* fix another warning
* fix warnings
* - fix warning
* - refactored test library and improved API
  - adapted alica tests
  - minor improved engine
* added friend declaration towards AlicaTestSupportUtility
  added dependency towards alica_test_support
* Merge pull request `#103 <https://github.com/rapyuta-robotics/alica/issues/103>`_ from rapyuta-robotics/repair-tests
  Repair tests
* Merge pull request `#107 <https://github.com/rapyuta-robotics/alica/issues/107>`_ from rapyuta-robotics/featureCommonConfs
  Feature common confs
* Merge branch 'repair-tests' into featureCommonConfs
* - changed c++ std to c++17
* - removed unused variable
* - finished unit test for reading configurations (now also for plans and plantypes)
* - implemented unit test for configrations on behaviours
  - adapted running plan for storing configurations for plantypes and plans, too
* - added states into sub plans (fix for invalid assignment problem in task assignment)
  - adapted behaviourpool to init basic behaviours with configurations
* - added plan for testing configurations (WIP)
  - fixed debugoutput include in header file of query (makes problems elsewhere)
* - implemented Configuration, ConfAbstractPlanWrapper
  - removed artifact from PlanningProblem class (some forward declaration and header files)
  - extended parser of the engine for the ConfAbstractWrapper entries in json files
* - fixed doubled included variable bindings
* - ported json format without confs to json format with confs
* - removed parameters from behaviours
* - increased available time for test execution from 60 to 300 sec
* - added layout file for attack plan
* - adapted time for tests
* - removed unnecessary debug output
* - removed debug output from AlicaContext
  - improved debug output in Synchronisation Process
  - fixed multiple entries in SyncRow's ReceivedBy list
  - fixed segfault/freez/undefined behaviour after a successfull synchronisation (deletion of SynchProcesses is now always done in SyncModule.tick())
* - put debug output into canonical alica_debug_message() macros
  - put a mutex in the destructor of SyncProcess
* - tried to fix synchronisation module: Test works, but synch protocol/module seems to be completely broken
* - renamed robotID to agentID
  - improved debug output
* - fix all but synchronisation test
* - fixed initialisation dependency
  - renamed method in alica context
* - made the alica_tests compile under json format
  - current issue: initialisation order of ALICA Engine -> things get messy, need a more proper solution here...
* - fix initialisation order for unit tests
* - removed unnecessary includes
  - reduced timeout for running alica_tests successful from 600 to 20 seconds
* Merge pull request `#86 <https://github.com/rapyuta-robotics/alica/issues/86>`_ from dasys-lab/integrate-auto-discovery
  New JSON Plan Format
* - fix for essentials include
* - restored alica_tests
* Merge branch 'rapyuta-robotics-v0.9.0' into integrate-auto-discovery
* Merge branch 'v0.9.0' of https://github.com/rapyuta-robotics/alica into rapyuta-robotics-v0.9.0
* Merge branch 'rr-devel' into bb_race_fix
* Merge branch 'rr-devel' into catkin_install_build
* Merge pull request `#75 <https://github.com/rapyuta-robotics/alica/issues/75>`_ from rapyuta-robotics/wb_store_id_option
  Wb store id option
* Casting, id type, naming & missing include fixes
  1. Change unsigned long long to uint64_t
  2. Use numeric_limits<uint64_t>::max() to get the max value
  3. Use static_cast instead of c-style cast to invoke conversion function
  4. Change persist_id to persistId as per naming convention
  5. Use uint64_t for id in test cases, otherwise it will fail
  6. Remove extra inclusion of header <random>
* Merge pull request `#82 <https://github.com/rapyuta-robotics/alica/issues/82>`_ from rapyuta-robotics/ctx_clock_api
  Provide api in AlicaContext to use a custom clock
* Provide api in AlicaContext to use a custom clock
  A custom clock is useful for simulations & testing in order to control the
  speed of the simulation. This clock can be set in the AlicaContext and will
  be used by the engine for time-based work.
* Merge pull request `#78 <https://github.com/rapyuta-robotics/alica/issues/78>`_ from rapyuta-robotics/v1.0.0
  Merge branch v1.0.0 into rr-devel
* - remove int type
* - minor fixes for tests (the need to be reimplemented)
* fixes for ID
* Merge pull request `#69 <https://github.com/rapyuta-robotics/alica/issues/69>`_ from rapyuta-robotics/localconf_improve
  Improve Local.conf to not depend on host name and add new context api's
* Improve Local.conf to not depend on host name and add new context api's
* Merge pull request `#1 <https://github.com/rapyuta-robotics/alica/issues/1>`_ from rapyuta-robotics/rr-devel
  Update from RR
* Merge pull request `#66 <https://github.com/rapyuta-robotics/alica/issues/66>`_ from rapyuta-robotics/auto_discovery
  Enable auto discovery of agents
* Change role to role id in message
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* - moved AgentIDConstPtr
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* Add discovery test case and fix review comments
* Fix test cases
* Enable auto discovery of agents
* Merge pull request `#65 <https://github.com/rapyuta-robotics/alica/issues/65>`_ from rapyuta-robotics/fix/crash-on-role-not-found
  Do not prevent an agent from being removed due to state protection timer
* test for interrupting communication between robots
* Fix minor racecondition in test (`#61 <https://github.com/rapyuta-robotics/alica/issues/61>`_)
  * fix race-condition in test
  * fix bad merge
  * fix merging woes
* Merge pull request `#60 <https://github.com/rapyuta-robotics/alica/issues/60>`_ from rapyuta-robotics/new_api
  New api for alica
* Added const getters in engine
* supple -> essential
* Merge branch 'master' into newPD
* Fix duplicate communicator startup and change function ordering
* Remove virtual for overridded methods in test
* Fixed test refactoring issues
* Refactor tests to use context
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
* Add Alica Context api class
* Merge pull request `#55 <https://github.com/rapyuta-robotics/alica/issues/55>`_ from rapyuta-robotics/system_config_fix
  Cleanup system config caching
* Cleanup system config caching
* Split tests into alica and supplementary (`#54 <https://github.com/rapyuta-robotics/alica/issues/54>`_)
  * Move engine related tests to alica
  * Missed file
  * Extend alica dummy proxy for inproc communication
  * Renamed variable
  * Remove ros proxy conf
  * formatting
  * Fix review comments
  * Another review comment
  * Moved varsync test to alica
* Merge pull request `#48 <https://github.com/rapyuta-robotics/alica/issues/48>`_ from rapyuta-robotics/new_alica
  WIP to move to 3 repo structure
* Further sync with new supplementary and essentials
* Renamed supplementary to essentials namespace for moved components
* Merge branch 'newPD' of github.com:carpe-noctem-cassel/alica into newPD
* - removed alica_tests and placed it into alica-supplementary
* fixed includes
  fixed double lid declaration
* added constraint solver
* - removed test for gSolver and put it (not ready yet) into the alica-supplementary repo
* - added dummy solver
* Minor bugfix (`#46 <https://github.com/rapyuta-robotics/alica/issues/46>`_)
  * make sure flags are set before notifications go out
  * duplicate include
  * improved handling of deepest node pointer
  * wait for query to be finished before testing result
* - reformat
* - renamed getVariableByName into getVariable and the BasicBehaviour now calls Behaviour::getVariable(..) instead of implemting the same again
* Merge pull request `#12 <https://github.com/rapyuta-robotics/alica/issues/12>`_ from carpe-noctem-cassel/StefanJakob-patch-1
  Update DistBallRobot.cpp
* Update DistBallRobot.cpp
  fixed missing std
* fixed alica tests
* Merge branch 'master' into newPD
* Merge pull request `#11 <https://github.com/rapyuta-robotics/alica/issues/11>`_ from rapyuta-robotics/rr_to_upstream
  Refactored Assignment and RunningPlan interfaces
* Merge branch 'master' into newPD
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
* Merge remote-tracking branch 'origin/rr-devel' into ab_fix_bugs_found_with_coverity
  # Conflicts:
  #	alica_engine/src/engine/Assignment.cpp
  #	alica_engine/src/engine/BasicBehaviour.cpp
  #	alica_tests/src/test/test_alica_behaviourtrigger.cpp
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
* quick fix
* Merge remote-tracking branch 'origin/hs_improve_query' into ab_fix_bugs_found_with_coverity
* Merge branch 'rr-devel' into hs_improve_query
* address PR comments
* Improve Alica Tests (`#16 <https://github.com/rapyuta-robotics/alica/issues/16>`_)
  * Add signal handler, change assert to expect, use step engine for synchs
  * Revert assert changes and have signal handler catch sigsegv
  * Start with no signal handling for now.
  * Remove unnecessary tab
  * Fix formatting
  * Deduplicate code
  * Have individual tests fail in the event of SIGSEGV
  * Simple one-line addition to tests for SIGSEGV handling
  * Test issue with travis.yml
  * Try -a instead of -k
  * Revert "Try -a instead of -k"
  This reverts commit 794837d064a1017cd867330940db40a4caef2314.
  * Revert "Test issue with travis.yml"
  This reverts commit 07d6000f2409bcd81feec4811f67ba010ac32ba1.
  * Print statements
  * Fix quotation escaping
  * test wstool up, not wstool merge
  * try -a instead of -k
  * catch SIGABRTs from failed asserts
  * Address first round of pull request comments
  * Use macro to replace setjmp command
* minor fixes and give more time to behavior trigger test
* increase test timeout
* fix condition
* make sure agents know each other in the test first
* add variable handling test
* fix unit tests
* fix range accessors
* rename
* Merge pull request `#7 <https://github.com/rapyuta-robotics/alica/issues/7>`_ from rapyuta-robotics/hs_constify_model
  Constify the Alica model
* rename Sets to Grps
* Merge branch 'rr-devel' into ab_travis_ci_fix
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
* test for variable syncing, clean up and additional static assert
* add blackboard test
* Variant tests
* minor fix
* fix test
* test compile
* fixes & use new interface in tests
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
* Merge branch 'hs_constify_model' into hs_solving_reloaded
* review comments
* fix missing sort predicate, add test
* clean planningProblem and destinationPath
* fix planwriter issues
* test improvement after fixing subvariable bug
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
* Merge branch 'mergein-kassel' into hs_constify_model
  # Conflicts:
  #	alica_engine/include/engine/RunningPlan.h
  #	alica_engine/include/engine/teammanager/TeamManager.h
  #	alica_engine/src/engine/RunningPlan.cpp
  #	alica_engine/src/engine/TeamObserver.cpp
  #	alica_engine/src/engine/model/ForallAgents.cpp
* fix bad merge
* better whitespaces
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
* - fixed nameing
* - tryed to fix wrong naming for alica_test(s)
  - removed old test package
* copied stuff from old alica_test to alica_tests
* - removed merge artifacts
* Merge branch 'rapyuta-robotics-hs_fix_unit_tests' into robotid
* Merge branch 'hs_fix_unit_tests' of https://github.com/rapyuta-robotics/alica into rapyuta-robotics-hs_fix_unit_tests
* - renamed alica_test to alica_tests
  - fixed task_assignment test although it does not really do anything
  - fixed some output errors of the alica logger (now ID instead of ID's address is written into log file)
* Contributors: Abhishek S, AbhishekS, Alexander Bolinsky, CNrobots, Dale Koenig, Hendrik, Hendrik Skubch, Luca Tricerri, Maksim Derbasov, Prajapati-Pawan, Rogerio Fonteles, Stefan Jakob, StefanJakob, StefanSchmelz, Stephan Opfer, Veeraj S Khokale, Witali Schmidt, athish-t, bjoernschroeder, bschroeder, christianhelm, dhananjay-patki, gajen, jironitta, mansiVerma26, veerajsk
