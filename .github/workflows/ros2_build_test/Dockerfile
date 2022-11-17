# syntax=docker/dockerfile:1.4

FROM ros:humble
SHELL ["/bin/bash", "-c"]

WORKDIR /root/ws/src

RUN <<deps
  apt-get update
  apt-get install -y libboost-all-dev
  rosdep install -y --from-paths . --ignore-src --rosdistro noetic
deps

COPY . src

RUN <<build
 source /opt/ros/humble/setup.bash
 colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests alica_msgs
build
