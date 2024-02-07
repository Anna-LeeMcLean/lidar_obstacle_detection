/* \author Anna-Lee McLean */
// Filtering, segmenting and clustering streams of point cloud data from a LiDAR for obstacle detection 
// for a self-driving car driving through a city.

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
    ProcessPointClouds<pcl::PointXYZI>* processor = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = processor->loadPcd("../data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, pointCloud, "inputCloud");

    // Filter point cloud using voxel grids
    float filterRes = 0.35;   // voxel grid size -> 0.35 x 0.35 x 0.35 (cm)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = processor->FilterCloud(pointCloud, filterRes, Eigen::Vector4f (-10.0, -5.0, -3.0, 1), Eigen::Vector4f (25.0, 6.5, 0.3, 1));
    //renderPointCloud(viewer, cloudFiltered, "filteredCloud");

    // Create point processor
    int maxIterations = 100;
    float distanceThreshold = 0.2;
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> point_cloud_pair;

    point_cloud_pair = processor->SegmentPlane(cloudFiltered, maxIterations, distanceThreshold);

    // Render plane and obstacle point clouds
    renderPointCloud(viewer, point_cloud_pair.first, "plane_point_cloud", Color(0.0, 1.0, 0.0));
    renderPointCloud(viewer, point_cloud_pair.second, "obstacle_point_cloud", Color(1.0, 0.0, 0.0));

    // Create clusters within the obstacle point cloud to distinguish the individual obstacles (cars and other objects of the sidewalk)
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> distinct_obstacle_point_clouds = processor->Clustering(point_cloud_pair.second, 0.7, 10, 250);

    int cluster_id = 0;

    for (const auto cluster : distinct_obstacle_point_clouds){
        std::cout << "cluster size: ";
        processor->numPoints(cluster);
        float c1 = (float) rand() / (RAND_MAX);
        float c2 = (float) rand() / (RAND_MAX);
        float c3 = (float) rand() / (RAND_MAX);
        renderPointCloud(viewer, cluster, "obstacle"+std::to_string(cluster_id), Color(c1, c2, c3));

        // Add bounding boxes around each of the distinct obstacle point clouds
        Box clusterBox = processor->BoundingBox(cluster);
        renderBox(viewer, clusterBox, cluster_id, Color(c1, c2, c3), 0.5);
        ++cluster_id;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}