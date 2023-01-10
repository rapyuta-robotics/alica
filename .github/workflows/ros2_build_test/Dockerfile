# syntax=docker/dockerfile:1.4

FROM ros:humble
SHELL ["/bin/bash", "-c"]


WORKDIR /root/ws/src

COPY . src

RUN <<deps
  apt-get update || exit 1
  apt-get install -y libboost-all-dev || exit 1
  source /opt/ros/humble/setup.bash || exit 1
  rosdep update || exit 1
  rosdep install -y -r --from-paths . --ignore-src --rosdistro humble
deps



RUN <<build
  source /opt/ros/humble/setup.bash || exit 1
  colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests
build
