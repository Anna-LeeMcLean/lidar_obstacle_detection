# LiDAR Obstacle Detection - Self-Driving Car

This project implements the obstacle detection pipeline for a point cloud data stream which was generated from LiDAR scans. 
The pipeline includes **filtering**, **segmentation** and **clustering** of point clouds to identify the road and various 
obstacles in the environment. The point cloud data was generated from a car as it moved through a city block.

## Obstacle Detection Process
### Filtering
  The purpose of filtering is to reduce the overall number of points in an input point cloud to allow for faster 
  processing times. This is done by removing unneccessary points from the cloud and reducing the cloud resolution
  since high detail is not required for obstacle detection. The resolution is reduced using PCL's built-in voxel 
  grid reduction algorithim. To remove unneccessary points from the point cloud, a particular region of interest
  is considered. This region stretches 10 meters behind the car and 25 meters ahead. Points within the width of 
  the road are also included in this region. The aim is to only consider points within the immediate vicinity of
  the car to avoid unnecessary processing. Finally, the LiDAR scan usually also picks up points that represent 
  the roof of the car. These points are also filtered out since they are not useful for obstacle detection/avoidance.

### Segmentation
  The filtered point cloud data is then separated into two separate point clouds; one which represents the road 
  and another which holds the points for all obstacles. This separation is done because we don't want to avoid the 
  road (quite the opposite actually) but require knowledge of the location of all objects so that they can be avoided. 
  The segementation in this project is done by fitting a plane to the point cloud data using a custom **RANSAC algorithim**. 
  It iteratively chooses 3 random points in the data and fits a plane to them, then determines how many other points 
  are on that plane. The plane with the most points that fit to it wins!

### Clustering
  Now we know what are obstacles and what is the road. We want to determine the indivdual obstacles in the resulting
  obstacle point cloud. This is done using a custom 3D **KDTree** algorithim which works to find the points which are 
  nearest to each other within a certain tolerance. Bounding boxes are drawn around these clusters and there you have it! 
  A stream of point cloud data where every frame is segemented into road/obstacles with each individual
  obstacle identified.
  
Original Point Cloud Data Stream

![original point cloud data data stream](https://github.com/Anna-LeeMcLean/lidar_obstacle_detection/assets/60242931/60fb99bc-9856-4250-8529-60abe8fc53ad)

Processed Point Cloud Data Stream

![processed point cloud data stream](https://github.com/Anna-LeeMcLean/lidar_obstacle_detection/assets/60242931/50aef8db-d32f-4be6-8a4e-7c357d27cda7)

The processed region is visibly smaller than the original point cloud with its resolution lower for the purpose of fast processing. 
The dark circle in the center of the streams represents the car which is mounted with the LiDAR sensor. The road and obstacles have 
been segmented into green and red point clouds respectively and bounding boxes are drawn around each obstacle cluster detected.

Demonstration of Pipeline from Console Output

![lidar obstacle detection output](https://github.com/Anna-LeeMcLean/lidar_obstacle_detection/assets/60242931/5756a360-1874-46b3-9f60-6eb90b7bb4a8)

## Installation Instructions

### Requirements
- Ubuntu 20.4.6
- PCL 1.14
- C++ 20
- gcc 9.4.0

### Ubuntu Local Install

If necessary, install PCL (this command installs the latest version):

`sudo apt install libpcl-dev`

Then clone repo and run executable:

`git clone https://github.com/Anna-LeeMcLean/lidar_obstacle_detection`

`cd lidar_obstacle_detection`

`mkdir build && cd build`

`cmake ..`

`make`

`./environment`
