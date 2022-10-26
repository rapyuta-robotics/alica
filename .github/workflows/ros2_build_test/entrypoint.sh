#!/bin/sh -l
. /opt/ros/humble/setup.sh

colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests alica_msgs

# colcon test

# colcon test-result --verbose


if [ $? -eq 0 ]
  then exit 0
fi 

exit 1