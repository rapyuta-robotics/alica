# syntax=docker/dockerfile:1.4

FROM ros:humble
SHELL ["/bin/bash", "-c"]

WORKDIR /root/ws/src

COPY . src


RUN <<build
 source /opt/ros/humble/setup.bash
 rosdep update 
 rosdep install --from-paths . --ignore-src --rosdistro humble -r -y
 colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests alica_msgs
build
