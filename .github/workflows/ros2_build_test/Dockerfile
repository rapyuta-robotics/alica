# syntax=docker/dockerfile:1.4

FROM ros:humble

RUN <<build
 . /opt/ros/humble/setup.sh
 colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests alica_msgs
build
