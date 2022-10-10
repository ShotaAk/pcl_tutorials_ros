# Tutorials

Launch example node with following command:

```sh
$ roslaunch pcl_tutorials_ros example.launch \
    example:=EXAMPLE_NAME \
    number:=EXAMPLE_NUMBER \
    realsense:=SET_TRUE_TO_USE_REALSENSE \
    gazebo:=SET_TRUE_TO_RUN_GAZEBO_SIMULATION \
    use_file:=SET_TRUE_TO_LOAD_PCD_FILE file:=PCD_FILE_PATH
```

Please refer to
https://pcl.readthedocs.io/projects/tutorials/en/master/
for PCL details.

- [Filtering](#filtering)
- [KdTree](#kdtree)
- [Segmentation](#segmentation)

## Filtering

The Example code is [here](https://github.com/ShotaAk/pcl_tutorials_ros/blob/master/src/filtering.cpp).

References:
https://pcl.readthedocs.io/projects/tutorials/en/master/#filtering

### 0: Filtering a PointCloud using a PassThrough filter

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=0 gazebo:=true
```

![passthrough](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/passThrough.png)

### 1: Downsampling a PointCloud using a VoxelGrid filter

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=1 gazebo:=true
```

![downsampling](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/downsampling.png)

### 2: Removing outliers using a StatisticalOutlierRemoval filter

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=2 use_file:=true file:=table_scene_lms400.pcd
```

![statisticalOutlierRemoval](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/statisticalOutlierRemoval.png)

### 3: Projecting points using a parametric model

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=3 gazebo:=true
```

![projecting](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/projecting.png)

### 4: Extracting indices from a PointCloud

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=4 gazebo:=true
```

![extractingIndices](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/extractingIndices.png)

[back to page top](#tutorials)

## KdTree

The example code is [here](https://github.com/ShotaAk/pcl_tutorials_ros/blob/master/src/kdtree.cpp).

References:
https://pcl.readthedocs.io/projects/tutorials/en/master/#kdtree

```sh
roslaunch pcl_tutorials_ros example.launch example:=kdtree gazebo:=true
```

![kdtree](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/kdtree/kdtree.png)

[back to page top](#tutorials)

## Segmentation

The example code is [here](https://github.com/ShotaAk/pcl_tutorials_ros/blob/master/src/segmentation.cpp).

References:
https://pcl.readthedocs.io/projects/tutorials/en/master/#segmentation

### 0: Plane model segmentation

```sh
roslaunch pcl_tutorials_ros example.launch example:=segmentation number:=0 gazebo:=true
```

![plane_model](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/segmentation/plane_model.png)

### 1: Cylinder model segmentation

TBD

### 2: Euclidean Cluster Extraction

```sh
roslaunch pcl_tutorials_ros example.launch example:=segmentation number:=2 gazebo:=true
```

![euclideanClusterExtraction](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/segmentation/euclideanClusterExtraction.png)

[back to page top](#tutorials)
