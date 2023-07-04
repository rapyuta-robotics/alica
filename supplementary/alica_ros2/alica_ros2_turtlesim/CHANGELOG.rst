^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_ros2_turtlesim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Fix roleset name to match rst file name (`#452 <https://github.com/rapyuta-robotics/alica/issues/452>`_)
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
* Remove codegen tags and add dynamic exports for ROS2 turtlesim (`#407 <https://github.com/rapyuta-robotics/alica/issues/407>`_)
  * remove TAGS
  * fix documentation
  * Plans and behaviours modified
  * change name
  * added library for dynamic loading
  * fix format
  * fix dependances
  * turtlesim compile but runtime problems
  * fix action
  * documentation fix
  * fix crash
  * fix crash
  * fix format
  * fix turtlesim
  * cleanup
  * cleanup
  * fix documentation
  * Removing worldmodel step 001
  * Removing worldmodel step 002
  * Removing worldmodel step 003
  * minor
  * fix comments and rename entities
  * fix comments
  ---------
  Co-authored-by: ¨triccyx¨ <¨triccyx@gmail.com¨>
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
* Alica ros2 interfaces (`#345 <https://github.com/rapyuta-robotics/alica/issues/345>`_)
  * Created interfaces for ros2 for alica_msgs, alica_ros2_proxy, and alica_ros2_turtlesim
  * Changed to use rclcpp naming conventions
  * More naming conventions
  * Cleaned up comments/unused files
  * run precommit on new files
  * Uncomment warning
  * fix CI errors
  * Fix CI again
  * More CI fixes
  * Addressed PR comments
  * Travis CI fixes
  * Still fighting travis
  * Changed roleId to role_id in ros1 proxy
  * changed to use rosdep in CI
  * fighting CI
  * Modified docs & changed CI to remove entrypoint
  * Fixed typo in dockerfile & ran precommit
  * Moved ros1 and ros2 dependent packages to their own subfolders
  * added apt-get call to CI
  * fixed compile flags in new subdirectory
  * moved turtlesim library to new location
  * Added changes to alica turtlesim codegen to ros2 turtlesim
  * Changed path for alica_turtlesim_library cmake flags
  * moved supplementary_tests_library
  * Skip keys required for ros2 in ros1 build CI
  * added update to find turtlesim in ros2 ci
  * Fixed issue with turtlesim dep not being resolved
  * fix CI build error from not sourcing rosdistro
  * Addressed PR feedback
  * Fix alignment
  * Update supplementary/alica_msgs/CMakeLists.txt
  Co-authored-by: Dale Koenig <dale.koenig@rapyuta-robotics.com>
* Contributors: Dale Koenig, Luca Tricerri, dhananjay-patki, veerajsk
