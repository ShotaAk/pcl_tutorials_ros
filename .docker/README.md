
# Docker image for pcl_tutorials_ros

## Installation

The [osrf/rocker](https://github.com/osrf/rocker) tool
enables to run ROS GUI application on docker easily.

Install ROS environment then you can
`$ sudo apt install python3-rocker`.

## Launch nodes

```sh
$ rocker --x11 --net=host --privileged pcl_tutorials_ros:$ROS_DISTRO \
    roslaunch pcl_tutorials_ros example.launch realsense:=true
```

## Build package

```sh
# Clone this package
$ mkdir -p ~/tutorials_ws/src
$ git clone https://github.com/ShotaAk/pcl_tutorials_ros ~/tutorials_ws/src

# Build package
$ rocker --x11 --net=host --privileged \
    --volume ~/tutorials_ws:/root/overlay_ws \
    -- pcl_tutorials_ros:$ROS_DISTRO \
    catkin_make

# Launch nodes
$ rocker --x11 --net=host --privileged \
    --volume ~/tutorials_ws:/root/overlay_ws \
    -- pcl_tutorials_ros:$ROS_DISTRO \
    roslaunch pcl_tutorials_ros example.launch realsense:=true
```

## Build Docker image

```sh
$ cd pcl_tutorials_ros/.docker
$ ./build.sh melodic
...
Successfully tagged pcl_tutorials_ros:melodic
```

## References

- Dockerfile for ROS project:
  - https://github.com/rt-net/crane_plus/tree/1.1.0/.docker
- Why is `--privileged` necessary for rocker command:
  - https://github.com/osrf/rocker/issues/13
