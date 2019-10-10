# PCL Tutorials with ROS

This package executes [pointclouds.org's tutorials](http://www.pointclouds.org/documentation/tutorials/)
on ROS enviroments.

![top_image](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/top_image.png)

# Requirments

- ROS Melodic
- Gazebo

## Optional

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
cd ~/catkin_ws/src/pcl_tutorials_ros/samples
./download.sh
```

# Usage

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

## Use Sample .pcd file

Example: ./samples/table_scene_lms400.pcd

```bash
roslaunch pcl_tutorials_ros example.launch use_file:=true file:=table_scene_lms400.pcd
```

![pcd_file](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/pcd_file.png)

# Tutorials

Please refer to [Tutorials page](./doc/Tutorials.md).
