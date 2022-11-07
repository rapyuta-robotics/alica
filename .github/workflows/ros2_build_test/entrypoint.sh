#!/bin/sh -l
. /opt/ros/humble/setup.sh

sudo rosdep init 
rosdep update 
rosdep install --from-paths ./ -y --ignore-src

colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests

if [ $? -ne 0 ]
  then exit 1
fi

# colcon test

# colcon test-result --verbose


exit 0
