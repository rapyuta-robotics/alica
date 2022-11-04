# syntax=docker/dockerfile:1.4

FROM ros:noetic
SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws/src

RUN <<catkin_tools
  apt-get update
  apt-get install -y python3-catkin-tools
catkin_tools

RUN <<build
  cd /root/catkin_ws
  source /opt/ros/noetic/setup.bash
  catkin init
  catkin config --blacklist alica_tracing --cmake-args -DCMAKE_CXX_FLAGS=-Werror
  catkin build --no-status
build

RUN <<runtests
  source ../devel/setup.bash
  catkin run_tests --no-status && catkin_test_results --verbose
runtests

# TODO: Coverity in gha
# if [ \"$TRAVIS_BRANCH\" = \"coverity_scan\" ] || [ \"$TRAVIS_EVENT_TYPE\" = \"cron\" ] ; then
#     wget -qO- https://scan.coverity.com/download/cxx/linux64
#         --post-data \"token=$COVERITY_TOKEN&project=rapyuta-robotics/$COVERITY_PROJECT\" |
#     tar xvz &&
#     python3 /travis/generate_coverity-analysis_script.py . &&
#     chmod +x coverity-analysis.sh &&
#     ./cov-analysis*/bin/cov-build --dir cov-int ./coverity-analysis.sh &&
#     tar czvf cs.tgz cov-int &&
#     curl
#         --form token=\"$COVERITY_TOKEN\"
#         --form email=\"$COVERITY_EMAIL\"
#         --form file=@cs.tgz
#         --form version=\"1.0\"
#         --form description=\"$COVERITY_PROJECT\"
#         https://scan.coverity.com/builds?project=rapyuta-robotics%2F$COVERITY_PROJECT ;
# fi"
