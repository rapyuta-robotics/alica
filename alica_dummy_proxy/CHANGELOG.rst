^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_dummy_proxy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Cleanup (`#426 <https://github.com/rapyuta-robotics/alica/issues/426>`_)
* Thread sanitizer fixes (`#400 <https://github.com/rapyuta-robotics/alica/issues/400>`_)
* Fixes to properly free allocated memory at shutdown (`#388 <https://github.com/rapyuta-robotics/alica/issues/388>`_)
  * Memory leak fix
  * Stop communication in dtor
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
* Fix dummy tracing in test (`#356 <https://github.com/rapyuta-robotics/alica/issues/356>`_)
  * Fix dummy tracing in test
  * Also deregister on stopCommunication
* Run precommit on all files (`#340 <https://github.com/rapyuta-robotics/alica/issues/340>`_)
* Removed Redundant cmake Libraries (`#336 <https://github.com/rapyuta-robotics/alica/issues/336>`_)
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
* Install in CI (`#235 <https://github.com/rapyuta-robotics/alica/issues/235>`_)
  * Install mode in ci
  * Fix alica dummy proxy install
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
* Merge branch 'rr-devel' into doxygen_integration
* Merge branch 'rr-devel' into gh-pages
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
* Merge pull request `#106 <https://github.com/rapyuta-robotics/alica/issues/106>`_ from rapyuta-robotics/json-plan-format
  Json plan format
* Merge pull request `#103 <https://github.com/rapyuta-robotics/alica/issues/103>`_ from rapyuta-robotics/repair-tests
  Repair tests
* Merge pull request `#107 <https://github.com/rapyuta-robotics/alica/issues/107>`_ from rapyuta-robotics/featureCommonConfs
  Feature common confs
* Merge branch 'v0.9.0' into featureCommonConfs
* Merge branch 'repair-tests' into featureCommonConfs
* - removed conversion via toStandard, because default copy constructor does the job
* Merge pull request `#105 <https://github.com/rapyuta-robotics/alica/issues/105>`_ from rapyuta-robotics/dk_install_fixes
  CMake install fixes
* Install headers
* CMake install fixes
* - tried to fix synchronisation module: Test works, but synch protocol/module seems to be completely broken
* - renamed robotID to agentID
  - improved debug output
* Merge pull request `#86 <https://github.com/rapyuta-robotics/alica/issues/86>`_ from dasys-lab/integrate-auto-discovery
  New JSON Plan Format
* - restored alica_tests
* Merge branch 'rapyuta-robotics-v0.9.0' into integrate-auto-discovery
* Merge branch 'v0.9.0' of https://github.com/rapyuta-robotics/alica into rapyuta-robotics-v0.9.0
* fixes for ID
* Merge pull request `#1 <https://github.com/rapyuta-robotics/alica/issues/1>`_ from rapyuta-robotics/rr-devel
  Update from RR
* Merge pull request `#66 <https://github.com/rapyuta-robotics/alica/issues/66>`_ from rapyuta-robotics/auto_discovery
  Enable auto discovery of agents
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* - moved AgentIDConstPtr
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* Enable auto discovery of agents
* Merge pull request `#60 <https://github.com/rapyuta-robotics/alica/issues/60>`_ from rapyuta-robotics/new_api
  New api for alica
* Added const getters in engine
* Merge branch 'master' into newPD
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
* - reformat
* Merge branch 'master' into newPD
* Merge pull request `#10 <https://github.com/rapyuta-robotics/alica/issues/10>`_ from rapyuta-robotics/rr_to_upstream
  Rewrite Autodiff, improve solver interface
* Merge pull request `#9 <https://github.com/rapyuta-robotics/alica/issues/9>`_ from rapyuta-robotics/hs_improve_query
  Improvements to query API
* Add a common config package (`#21 <https://github.com/rapyuta-robotics/alica/issues/21>`_)
  * add common compiler configs
  * add dependencies to common config
  * formatting
* Merge pull request `#8 <https://github.com/rapyuta-robotics/alica/issues/8>`_ from carpe-noctem-cassel/robotid
  Robotid
* Merge pull request `#7 <https://github.com/rapyuta-robotics/alica/issues/7>`_ from rapyuta-robotics/hs_constify_model
  Constify the Alica model
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
* test compile
* Some improvements (`#6 <https://github.com/rapyuta-robotics/alica/issues/6>`_)
  * clean up
  * formatting changes
  * first simplification iteration on RunningPlan
* formatting changes
* Merge pull request `#3 <https://github.com/rapyuta-robotics/alica/issues/3>`_ from rapyuta-robotics/clang5_formatter
  Clang5 formatter and applied formating
* clang5 formatter changes
* removed old style
* alica test refactoring and bug fixing
* fix warning
* Revert "some warnings fixed"
  This reverts commit ee9d141ea90398db812e1f9ca712333fb6eb398b.
* some warnings fixed
* fix and cleanup cmake for kinetic
* update cmakelist.txt and removed rqt
* fix infinite recursion
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* renamed BehaviourEngineInfo into AlicaEngineInfo
* fixed dependencies
* refactored typedef for time
* Merge branch 'master' of github.com:carpe-noctem-cassel/alica
* Bug fix by Paul
* Communication dummy proxy and systemclock for alica
* Contributors: Abhishek S, AbhishekS, Alex, Alexander Bolinsky, Brain2, Christoph Eickhoff, Dale Koenig, Gautham Manoharan, Hendrik, Hendrik Skubch, Luca Tricerri, Maksim Derbasov, Stefan Jakob, StefanJakob, StefanSchmelz, Stephan Opfer, athish-t, bjoernschroeder, bschroeder, dhananjay-patki, jironitta, mansiVerma26, ppa
