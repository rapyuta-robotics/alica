#!/bin/bash
set -ev

BRANCH=$1
BASE_BRANCH=$2
REGEX='^.*version:[[:blank:]]*([^[:blank:]]*)[[:blank:]]*$'
ORIGINAL_BRANCH="NO_MATCH"

while read LINE ; do 
    if [[ $LINE =~ $REGEX ]] ; then
        ORIGINAL_BRANCH=${BASH_REMATCH[1]} 
        break 
    fi
done < ../dependencies.rosinstall 


cmd="bash -c \"
    cd ~/catkin_ws/src &&
    wstool init &&
    if [[ -f /travis/dependencies.rosinstall ]] ; then
        if [[ $ORIGINAL_BRANCH != \"NO_MATCH\" ]] ; then
            sed -i -e \"s/$ORIGINAL_BRANCH/$BRANCH/g\" /travis/dependencies.rosinstall ;
            wstool merge -a -y /travis/dependencies.rosinstall ;
        fi
        if ! wstool up ; then
            sed -i -e \"s/$BRANCH/$BASE_BRANCH/g\" /travis/dependencies.rosinstall ;
            wstool merge -a -y /travis/dependencies.rosinstall ;
            wstool up ;
        fi ;
        if ! wstool up ; then
            sed -i -e \"s/$BASE_BRANCH/$ORIGINAL_BRANCH/g\" /travis/dependencies.rosinstall ;
            wstool merge -a -y /travis/dependencies.rosinstall ;
            wstool up ;
        fi ;
    fi &&
    cd ~/catkin_ws &&
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO\""

docker_cmd="docker exec -t alica-test $cmd"

echo "running command: \"$docker_cmd\""
eval $docker_cmd
