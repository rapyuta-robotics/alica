#!/bin/sh -l
. /opt/ros/humble/setup.sh

colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests alica_msgs

if [ $? -ne 0 ]
  then exit 1
fi

# colcon test

# colcon test-result --verbose


exit 0
