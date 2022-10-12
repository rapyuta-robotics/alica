#!/bin/sh -l
. /opt/ros/humble/setup.sh

colcon build --packages-skip alica_msgs alica_ros_proxy alica_tracing constraintsolver alica_ros_turtlesim alica_tests supplementary_tests 2> result.txt

# colcon test

# colcon test-result --verbose

cat result.txt


if grep -q "Failed" result.txt ; then
  echo "Build Failed!"
  exit 1
fi

exit 0