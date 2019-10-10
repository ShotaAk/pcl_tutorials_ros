# Tutorials

Please refer to http://www.pointclouds.org/documentation/tutorials/

# Filtering

## Filtering a PointCloud using a PassThrough filter

http://www.pointclouds.org/documentation/tutorials/passthrough.php#passthrough

```bash
roslaunch my_pcl_tutorial example.launch gazebo:=true number:=0
```

## Downsampling a PointCloud using a VoxelGrid filter

http://www.pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid

```bash
roslaunch my_pcl_tutorial example.launch gazebo:=true number:=1
```

## Removing outliers using a StatisticalOutlierRemoval filter

http://www.pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal

```bash
roslaunch my_pcl_tutorial example.launch sample:=true file:=table_scene_lms400.pcd number:=2
```
