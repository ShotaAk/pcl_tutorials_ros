# Tutorial package for PCL with ROS

This package executes [pointclouds.org's tutorials](http://www.pointclouds.org/documentation/tutorials/)
on ROS enviroments.

# Requirments

- ROS Melodic
- Depth Camera
    - Currently support Realsense D415

# Installation

```bash
# Clone this package
cd ~/catkin_ws/src
git clone https://github.com/ShotaAk/pcl_tutorials_ros

# for Realsense
git clone https://github.com/IntelRealSense/realsense-ros

# Install packages
rosdep install -r -y --from-paths . --ignore-src

# Build
cd ~/catkin_ws
catkin_make

# Download sample pcd files
sudo apt install wget
cd ~/catkin_ws/src/my_pcl_tutorial/samples
./download.sh
```

# Usage

## Use Realsense D415

```bash
roslaunch my_pcl_tutorial example.launch realsense:=true
```

## Use Gazebo

```bash
roslaunch my_pcl_tutorial example.launch gazebo:=true
```

## Use Sample .pcd file

Example: ./samples/table_scene_lms400.pcd

```bash
roslaunch my_pcl_tutorial example.launch use_file:=true file:=table_scene_lms400.pcd
```

# Tutorials

Please refer to [Tutorial page](./doc/Tutorials.md).
