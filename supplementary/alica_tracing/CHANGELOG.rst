^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_tracing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Allow taking tracing tags from env (`#470 <https://github.com/rapyuta-robotics/alica/issues/470>`_)
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Fix Alica tracing error marking (`#447 <https://github.com/rapyuta-robotics/alica/issues/447>`_)
  fix tracing mark error description handling
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
* Minor CMake bug in alica_tracing (`#377 <https://github.com/rapyuta-robotics/alica/issues/377>`_)
* Allow using basic types in traces (`#372 <https://github.com/rapyuta-robotics/alica/issues/372>`_)
  * allow using basic types in traces
  * fix formatting and merge errors
  * add kv logs
  * fix formatting
* Fix warnings from Wextra and Wpedantic (`#364 <https://github.com/rapyuta-robotics/alica/issues/364>`_)
  * fix wpedantic
  * fix Wextra
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Traces tag time when closed (`#359 <https://github.com/rapyuta-robotics/alica/issues/359>`_)
  * Added tags when traces are finished/destroyed
  * changed tag to unix timestamp
  * changed to use ms in end time tag
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
* Exporting jaeger (`#319 <https://github.com/rapyuta-robotics/alica/issues/319>`_)
* Prompt sudo and relax gcc version (`#252 <https://github.com/rapyuta-robotics/alica/issues/252>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Update cmake version (`#237 <https://github.com/rapyuta-robotics/alica/issues/237>`_)
* Fix cmake install (`#234 <https://github.com/rapyuta-robotics/alica/issues/234>`_)
* Format line to eof (`#222 <https://github.com/rapyuta-robotics/alica/issues/222>`_)
  * Add line at eof in format
  * Dont auto add line
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
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
* Fix broken links in readme's (`#197 <https://github.com/rapyuta-robotics/alica/issues/197>`_)
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
* Contributors: Abhishek S, Ashwary Anand, Dale Koenig, David Simões, Dzmitry Ivashniou, Luca Tricerri, bjoernschroeder, dhananjay-patki, veerajsk
