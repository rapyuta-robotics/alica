# syntax=docker/dockerfile:1.4

FROM ros:lyrical-ros-base
SHELL ["/bin/bash", "-c"]

WORKDIR /root/ws

COPY . src/alica

RUN <<deps
  apt-get update || exit 1
  apt-get install -y libboost-all-dev || exit 1
  source /opt/ros/lyrical/setup.bash
  rosdep update || exit 1
  rosdep install -y -r --from-paths src --ignore-src --rosdistro lyrical
deps

RUN <<build
  source /opt/ros/lyrical/setup.bash
  colcon build --event-handlers console_direct- --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim supplementary_tests || exit 1
build

RUN <<runtests
  source install/setup.bash
  colcon test --event-handlers console_direct- --return-code-on-test-failure --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim supplementary_tests --ctest-args --output-on-failure --timeout 600
  status=$?
  colcon test-result --verbose
  exit $status
runtests
