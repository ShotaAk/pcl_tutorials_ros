# Tutorials

Please refer to
https://pcl.readthedocs.io/projects/tutorials/en/master/
for details.

- [Filtering](#filtering)
- [Segmentation](#segmentation)
- [KdTree](#kdtree)

## Filtering

The Example code is [here](https://github.com/ShotaAk/pcl_tutorials_ros/blob/master/src/filtering.cpp).

References:
https://pcl.readthedocs.io/projects/tutorials/en/master/#filtering

### Filtering a PointCloud using a PassThrough filter

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=0 gazebo:=true
```

![passthrough](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/passThrough.png)

### Downsampling a PointCloud using a VoxelGrid filter

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=1 gazebo:=true
```

![downsampling](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/downsampling.png)

### Removing outliers using a StatisticalOutlierRemoval filter

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=2 use_file:=true file:=table_scene_lms400.pcd
```

![statisticalOutlierRemoval](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/statisticalOutlierRemoval.png)

### Projecting points using a parametric model

```sh
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=3 gazebo:=true
```

![projecting](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/filtering/projecting.png)

### Extracting indices from a PointCloud

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

### Plane model segmentation

```sh
roslaunch pcl_tutorials_ros example.launch example:=segmentation number:=0 gazebo:=true
```

![plane_model](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/segmentation/plane_model.png)

### Cylinder model segmentation

TBD

### Euclidean Cluster Extraction

```sh
roslaunch pcl_tutorials_ros example.launch example:=segmentation number:=2 gazebo:=true
```

![euclideanClusterExtraction](https://github.com/ShotaAk/pcl_tutorials_ros/raw/images/segmentation/euclideanClusterExtraction.png)

[back to page top](#tutorials)
