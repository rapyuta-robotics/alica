# syntax=docker/dockerfile:1.4

FROM ros:noetic
SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws/src/alica/

COPY . .

WORKDIR /root/catkin_ws/

RUN <<deps
  apt-get update
  apt-get install -y python3-catkin-tools libboost-all-dev
  rosdep install -y --from-paths . --ignore-src --rosdistro noetic
deps

RUN <<build
  source /opt/ros/noetic/setup.bash
  catkin init
  catkin config --skiplist alica_tracing
  catkin build --no-status
build

RUN <<runtests
  source devel/setup.bash
  catkin run_tests --no-status
  catkin_test_results --verbose
runtests
