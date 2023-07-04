^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package alica_ros2_proxy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2023-07-04)
------------------
* Removing svn obsolete stuff from CMakeLists (`#486 <https://github.com/rapyuta-robotics/alica/issues/486>`_)
  * Removing svn obsolete from CMakeLists
  * Review round
* Update version numbers to prepare for 1.0.0 tag (`#467 <https://github.com/rapyuta-robotics/alica/issues/467>`_)
  Version 1.0.0
* Cleanup (`#426 <https://github.com/rapyuta-robotics/alica/issues/426>`_)
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
* Contributors: Dale Koenig, Maksim Derbasov, dhananjay-patki
