^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_solver_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Put all tests behind BUILD_TESTING cmake flag (`#346 <https://github.com/rapyuta-robotics/alica/issues/346>`_)
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
* Format line to eof (`#222 <https://github.com/rapyuta-robotics/alica/issues/222>`_)
  * Add line at eof in format
  * Dont auto add line
  Co-authored-by: Abhishek S <abhishek.sharma@rapyuta-robotics.com>
* Change cmake version (`#170 <https://github.com/rapyuta-robotics/alica/issues/170>`_)
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Merge pull request `#106 <https://github.com/rapyuta-robotics/alica/issues/106>`_ from rapyuta-robotics/json-plan-format
  Json plan format
* Merge pull request `#86 <https://github.com/rapyuta-robotics/alica/issues/86>`_ from dasys-lab/integrate-auto-discovery
  New JSON Plan Format
* Merge branch 'rapyuta-robotics-v0.9.0' into integrate-auto-discovery
* Merge branch 'v0.9.0' of https://github.com/rapyuta-robotics/alica into rapyuta-robotics-v0.9.0
* Merge branch 'rr-devel' into bb_race_fix
* Merge pull request `#83 <https://github.com/rapyuta-robotics/alica/issues/83>`_ from rapyuta-robotics/catkin_install_build
  Add install targets for catkin install build
* Add install targets for catkin install build
* Merge branch 'newPD' of github.com:dasys-lab/alica into newPD
* Merge branch 'new_api' of github.com:rapyuta-robotics/alica into new_api
* Revert "move to c++14, clean cmakelist files"
  This reverts commit 4ee14c88f6e460921f704980f718dd56265ecde3.
* Merge branch 'rr-devel' into new_api
* Switch to C++14 (`#59 <https://github.com/rapyuta-robotics/alica/issues/59>`_)
  * move to c++14, clean cmakelist files
  * fix eclipse cpp version
* move to c++14, clean cmakelist files
* - reformat
* Merge branch 'master' into newPD
* Merge pull request `#11 <https://github.com/rapyuta-robotics/alica/issues/11>`_ from rapyuta-robotics/rr_to_upstream
  Refactored Assignment and RunningPlan interfaces
* Merge branch 'master' into newPD
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
* Contributors: Abhishek S, AbhishekS, Dale Koenig, Hendrik, Hendrik Skubch, Maksim Derbasov, StefanSchmelz, Stephan Opfer, Veeraj S Khokale, dhananjay-patki, jironitta
