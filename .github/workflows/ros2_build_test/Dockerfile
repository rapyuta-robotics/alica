# syntax=docker/dockerfile:1.4

FROM ros:humble
SHELL ["/bin/bash", "-c"]


WORKDIR /root/ws/src

COPY . src

RUN apt-get update && \
    apt-get install -y libboost-all-dev && \
    source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install -y -r --from-paths . --ignore-src --rosdistro humble



RUN source /opt/ros/humble/setup.bash && \
    colcon build --continue-on-error --packages-skip alica_ros_proxy alica_tracing alica_ros_turtlesim alica_tests supplementary_tests
