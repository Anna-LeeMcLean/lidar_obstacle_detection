# LiDAR Obstacle Detection - Self-Driving Car

This project implements the obstacle detection pipeline from a stream of point cloud data generated from LiDAR scans. 
The pipeline includes **filtering**, **segmentation** and **clustering** of points clouds to determine the road and various 
obstacles in the environment. The point cloud data was generated from a car as it moved through a city block.

## Filtering
  The raw point cloud data is downsampled using voxel grid reduction to reduce the resolution of the point cloud.
  This allows for faster processing during the future segmentation and clustering steps. The point cloud is also
  reduced to a particular region of interest that stretches 10 meteres behind the car and 25 meters ahead. Points
  within the width of the road aare also only considered. This point reduction further aids in speeding up the
  required processing time for the segmentation and clustering techniques. Finally, the LiDAR scan usually also
  picks up points that represent the roof of the car. These points are also filtered out since they are not useful
  for obstacle detection/avoidance.

## Segmentation
  The filtered point cloud data is then separated into two separate point clouds; one which represents the road the
  car is travelling on and another which holds the points for all obstacles. This separation is done because we don't
  want to avoid the road (quite the opposite actually) but require knowledge of the location of all objects so that
  they can be avoided. The segementation in this project is done by fitting a plane to the point cloud data using a
  custom **RANSAC algorithm**. It iteratively chooses 3 random points in the data and fits a plane to them, then determines
  how many other points are on that plane. The plane with the most points that fit to it wins!

## Clustering
  Now we know what are obstacles and what is the road. We want to determine the indivdual obstacles in the resulting
  obstacle point cloud froom segmentation. This is done using a custom 3D **KDTree** algorithim which works to find the
  nodes which are nearest to each other within a certain tolerance. Bounding boxes are drawn around these clusters and
  there you have it! A stream of poitn cloud data where every frame is segemented into road/obstacles with each individual
  obstacle identified.
