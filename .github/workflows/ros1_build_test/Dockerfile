# syntax=docker/dockerfile:1.4

FROM ros:noetic
SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws/src/alica/

COPY . .

WORKDIR /root/catkin_ws/

RUN apt-get update && \
    apt-get install -y python3-catkin-tools libboost-all-dev && \
    rosdep install -y --from-paths . --ignore-src --skip-keys='rclcpp ament_cmake' --rosdistro noetic


RUN source /opt/ros/noetic/setup.bash && \
    catkin init && \
    catkin config --install --skiplist alica_tracing alica_ros2_proxy alica_ros2_turtlesim && \
    catkin build --no-status


RUN source devel/setup.bash && \
    catkin run_tests --no-status && \
    catkin_test_results --verbose
