# Tutorials

Please refer to http://www.pointclouds.org/documentation/tutorials/

# Filtering

## Filtering a PointCloud using a PassThrough filter

http://www.pointclouds.org/documentation/tutorials/passthrough.php#passthrough

```bash
roslaunch my_pcl_tutorial example.launch example:=filtering number:=0 gazebo:=true
```

## Downsampling a PointCloud using a VoxelGrid filter

http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

```bash
roslaunch my_pcl_tutorial example.launch example:=filtering number:=1 gazebo:=true
```

## Removing outliers using a StatisticalOutlierRemoval filter

http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal

```bash
roslaunch my_pcl_tutorial example.launch example:=filtering number:=2 sample:=true file:=table_scene_lms400.pcd
```

## Projecting points using a parametric model

http://www.pointclouds.org/documentation/tutorials/project_inliers.php#project-inliers

```bash
roslaunch my_pcl_tutorial example.launch example:=filtering number:=3 gazebo:=true
```

## Extracting indices from a PointCloud

http://www.pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices

TBD

