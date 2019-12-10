# PCL Tutorials with ROS

![Workflow Status](https://github.com/ShotaAk/pcl_tutorials_ros/workflows/ROS-Melodic/badge.svg)

This package executes [pointclouds.org's tutorials](http://www.pointclouds.org/documentation/tutorials/)
on ROS environments.

![statisticalOutlierRemoval](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/filtering/statisticalOutlierRemoval.png)

# Requirments

- ROS Melodic
- Gazebo

## Optional

- RealSense
    - RealSense D415 
    - [RealSense SDK](https://github.com/IntelRealSense/librealsense)
    - [RealSense ROS Package](https://github.com/IntelRealSense/realsense-ros)

# Installation

```bash
# Clone this package
cd ~/catkin_ws/src
git clone https://github.com/ShotaAk/pcl_tutorials_ros

# Install package dependencies
rosdep install -r -y --from-paths . --ignore-src

# Build
cd ~/catkin_ws
catkin_make

# Download sample pcd files
sudo apt install wget
cd ~/catkin_ws/src/pcl_tutorials_ros/samples
./download.sh
```

# Usage

## Tutorials

Please refer to [Tutorials page](./doc/Tutorials.md).

## Use Realsense D415

```bash
roslaunch pcl_tutorials_ros example.launch realsense:=true
```

![realsense](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/realsense.png)

## Use Gazebo

```bash
roslaunch pcl_tutorials_ros example.launch gazebo:=true
```

![gazebo](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/gazebo.png)

## Use PCD file

Example: `./samples/pcl_logo.pcd`

```bash
roslaunch pcl_tutorials_ros example.launch use_file:=true file:=pcl_logo.pcd
```

![pcd_file](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/pcd.png)
