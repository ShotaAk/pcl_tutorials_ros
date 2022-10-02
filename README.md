# PCL Tutorials with ROS

[![industrial_ci](https://github.com/ShotaAk/pcl_tutorials_ros/actions/workflows/industrial_ci.yaml/badge.svg?branch=master)](https://github.com/ShotaAk/pcl_tutorials_ros/actions/workflows/industrial_ci.yaml)

This package executes [pointclouds.org's tutorials](https://pcl.readthedocs.io/projects/tutorials/en/master/)
on ROS environments.

![statisticalOutlierRemoval](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/statisticalOutlierRemoval.png)

## Requirments

- ROS Melodic

### Optional

- RealSense
  - RealSense D415
  - [RealSense SDK](https://github.com/IntelRealSense/librealsense)

## Docker image

Please refer to [.docker/README.md](.docker/README.md) for details.

```sh
docker pull ghcr.io/shotaak/pcl_tutorials_ros:melodic
```

## Source build

```sh
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

## Tutorials

Please refer to [pcl_tutorials_ros's Tutorials page](./doc/Tutorials.md).

### Use Realsense D415

```sh
roslaunch pcl_tutorials_ros example.launch realsense:=true
```

![realsense](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/realsense.png)

### Use Gazebo

```sh
roslaunch pcl_tutorials_ros example.launch gazebo:=true
```

![gazebo](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/gazebo.png)

### Use PCD file

Example: `./samples/pcl_logo.pcd`

```sh
roslaunch pcl_tutorials_ros example.launch use_file:=true file:=pcl_logo.pcd
```

![pcd_file](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/pcd.png)

## License

MIT License
