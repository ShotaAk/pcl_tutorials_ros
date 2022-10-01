#!/usr/bin/env bash

set -e

if [ $# -eq 0 ]; then
    echo "Please set ROS_DISTRO to the argument."
    echo "e.g. ./build_source.sh melodic"
fi
ROS_DISTRO=$1

DOCKER_OPTION=""
if [ $# -ge 2 ]; then
    DOCKER_OPTION=$2
fi

CATKIN_OPTION=""
if [ $# -ge 3 ]; then
    CATKIN_OPTION=$3
fi

# Move to the package route directory
cd $(dirname $0)/../
docker build $DOCKER_OPTION -t pcl_tutorials_ros:$ROS_DISTRO -f .docker/Dockerfile . \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg CATKIN_OPTION=$CATKIN_OPTION