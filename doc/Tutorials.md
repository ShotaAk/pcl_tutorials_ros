# Tutorials

Please refer to http://www.pointclouds.org/documentation/tutorials/

- [Filtering](#filtering)
- [Segmentation](#segmentation)
- [KdTree](#kdtree)

# Filtering

[back to page top](#tutorials)

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

TBD

# KdTree

[back to page top](#tutorials)

http://www.pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search

```bash
roslaunch pcl_tutorials_ros example.launch example:=kdtree gazebo:=true
```

![kdtree](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/kdtree/kdtree.png)

# Segmentation

[back to page top](#tutorials)

## Plane model segmentation

http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

```bash
roslaunch pcl_tutorials_ros example.launch example:=segmentation number:=0 gazebo:=true
```

![plane_model](https://github.com/ShotaAk/pcl_tutorials_ros/blob/images/segmentation/plane_model.png)
