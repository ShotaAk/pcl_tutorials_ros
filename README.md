# Tutorial package for PCL with ROS

# Requirments

- ROS Melodic
- Depth Camera
    - Currently support Realsense D415

# Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/ShotaAk/my_pcl_tutorial

# for Realsense
git clone https://github.com/IntelRealSense/realsense-ros

rosdep install -r -y --from-paths . --ignore-src

cd ~/catkin_ws
catkin_make

# Download sample pcd files
sudo apt install wget
cd ~/catkin_ws/src/my_pcl_tutorial/samples
./download.sh
```

# Usage

## Change sample function

```bash
roslaunch my_pcl_tutorial example.launch gazebo:=true numbder:=0

roslaunch my_pcl_tutorial example.launch gazebo:=true numbder:=1
```

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
roslaunch my_pcl_tutorial example.launch sample:=true file:=table_scene_lms400.pcd
```

# Tutorials

Please refer to [Tutorial page](./doc/Tutorials.md).
