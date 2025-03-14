# Global Obstacle Avoidance

The goal of this module is to detect obstacles, map them in a global map and find a reference line through it. Our idea was it to use this as initial solution for model predictive control, but we didn't manage to finish the MPC. 

## Topics

## SLAM
We are using the ORB-SLAM3 for precise localization, enabling us to create an acurate map.

## Object Detector

This Node takes an depth image input stream, converts it to a point cloud and filters the resulting point cloud using: Voxel Grid, RANSAC and euclidean clustering.

### Parameters

_debug_ransac: Debug message for RANSAC
_debug_voxel: Debug message for Voxel Grid
_debug_clustering: Debug message for clustering
_debug_projection: Debug message for projection

_minPercentage: How mouch of the point cloud should be filtered through ransac
_planeDistance: Max allowed distance form point to plane
_clusteringDistance: Max allowed clustering distance

## Path Finder

Path finding module based on an occupancy map representation and A*

### Parameter

groundFilter: Height of object points to be ignored
resolution: Resolution of occupancy grid
mapWidth: width of occupancy grid in m
mapHeight: lenght of occupancy grid in m

## Trajectory Planer

Interpolating found path on occupancy grid using cubic hermite.

### Parameter

trajResolution: Points per step in occupancy grid