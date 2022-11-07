#!/bin/sh -l
. /opt/ros/humble/setup.sh


rosdep update 
rosdep install --from-paths ./ --ignore-src -r -y

colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests

if [ $? -ne 0 ]
  then exit 1
fi

# colcon test

# colcon test-result --verbose


exit 0
