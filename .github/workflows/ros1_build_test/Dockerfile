# syntax=docker/dockerfile:1.4

FROM ros:noetic
SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws/src/alica/

COPY . .

WORKDIR /root/catkin_ws/

RUN <<deps
  apt-get update || exit 1
  apt-get install -y python3-catkin-tools libboost-all-dev || exit 1
  rosdep install -y --from-paths . --ignore-src --skip-keys='rclcpp ament_cmake' --rosdistro noetic
deps

RUN <<build
  source /opt/ros/noetic/setup.bash
  catkin init || exit 1
  catkin config --install --skiplist alica_tracing alica_ros2_proxy alica_ros2_turtlesim || exit 1
  catkin build --no-status
build

RUN <<runtests
  source devel/setup.bash
  GTEST_FILTER=-AlicaTurtlesimTest:* catkin run_tests --no-status || exit 1
  catkin_test_results --verbose
runtests
