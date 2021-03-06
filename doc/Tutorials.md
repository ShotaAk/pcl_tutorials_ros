# Tutorials

Please refer to http://www.pointclouds.org/documentation/tutorials/

- [Filtering](#filtering)
- [Segmentation](#segmentation)
- [KdTree](#kdtree)

# Filtering

[back to page top](#tutorials)

The Example code is [here](https://github.com/ShotaAk/pcl_tutorials_ros/blob/master/src/filtering.cpp).

## Filtering a PointCloud using a PassThrough filter

http://www.pointclouds.org/documentation/tutorials/passthrough.php#passthrough

```bash
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=0 gazebo:=true
```

![passthrough](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/filtering/passThrough.png)

## Downsampling a PointCloud using a VoxelGrid filter

http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

```bash
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=1 gazebo:=true
```

![downsampling](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/filtering/downsampling.png)

## Removing outliers using a StatisticalOutlierRemoval filter

http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal

```bash
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=2 use_file:=true file:=table_scene_lms400.pcd
```

![statisticalOutlierRemoval](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/filtering/statisticalOutlierRemoval.png)

## Projecting points using a parametric model

http://www.pointclouds.org/documentation/tutorials/project_inliers.php#project-inliers

```bash
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=3 gazebo:=true
```

![projecting](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/filtering/projecting.png)

## Extracting indices from a PointCloud

http://www.pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices

```bash
roslaunch pcl_tutorials_ros example.launch example:=filtering number:=4 gazebo:=true
```

![extractingIndices](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/filtering/extractingIndices.png)

# KdTree

[back to page top](#tutorials)

The example code is [here](https://github.com/ShotaAk/pcl_tutorials_ros/blob/master/src/kdtree.cpp).

http://www.pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search

```bash
roslaunch pcl_tutorials_ros example.launch example:=kdtree gazebo:=true
```

![kdtree](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/kdtree/kdtree.png)

# Segmentation

[back to page top](#tutorials)

The example code is [here](https://github.com/ShotaAk/pcl_tutorials_ros/blob/master/src/segmentation.cpp).

## Plane model segmentation

http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

```bash
roslaunch pcl_tutorials_ros example.launch example:=segmentation number:=0 gazebo:=true
```

![plane_model](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/segmentation/plane_model.png)

## Cylinder model segmentation

TBD

## Euclidean Cluster Extraction

http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

```bash
roslaunch pcl_tutorials_ros example.launch example:=segmentation number:=2 gazebo:=true
```

![euclideanClusterExtraction](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/segmentation/euclideanClusterExtraction.png)

