/* \author Anna-Lee McLean */
// Filtering, segmenting and clustering streams of point cloud data from a LiDAR for obstacle detection 
// for a self-driving car driving through a city.

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <unistd.h>

template<typename PointT>
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<PointT>* pointProcessor, const typename pcl::PointCloud<PointT>::Ptr& inputCloud){

    // Filter point cloud using voxel grids
    float filterRes = 0.17;   // voxel grid size -> 0.35 x 0.35 x 0.35 (cm)
    Eigen::Vector4f minPoint = Eigen::Vector4f(-10.0, -5.0, -2.0, 1);
    Eigen::Vector4f maxPoint = Eigen::Vector4f (25.0, 6.5, 0.3, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessor->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);

    // Segment road and obstacles using RANSAC plane segmentation
    int maxIterations = 100;
    float distanceThreshold = 0.18;
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> point_cloud_pair;

    point_cloud_pair = pointProcessor->Ransac(cloudFiltered, maxIterations, distanceThreshold);

    // Render segmented plane point cloud (road)
    renderPointCloud(viewer, point_cloud_pair.first, "plane_point_cloud", Color(0.0, 1.0, 0.0));
    //renderPointCloud(viewer, point_cloud_pair.second, "obstacle_point_cloud", Color(1.0, 1.0, 0.0));
    
    // Create clusters within the obstacle point cloud to distinguish the individual obstacles (cars and other objects on the sidewalk)
    float clusterTolerance = 0.41;
    int minClusterPoints = 17;
    int maxClusterPoints = 510;
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> distinct_obstacle_point_clouds = pointProcessor->EuclideanCluster(point_cloud_pair.second, clusterTolerance, minClusterPoints, maxClusterPoints);

    // Render clusters with bounding boxes around each
    int cluster_id = 0;
    for (const auto cluster : distinct_obstacle_point_clouds){
        std::cout << "cluster size: ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacle"+std::to_string(cluster_id), Color(0.0, 0.0, 1.0));

        // Add bounding boxes around each of the distinct obstacle point clouds
        Box clusterBox = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, clusterBox, cluster_id, Color(0.0, 0.0, 1.0), 0.5);
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

    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> pathStream = pointProcessor->streamPcd("../data/pcd/data_1/");
    auto streamIterator = pathStream.begin();
    typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    /*
    inputCloud = pointProcessor->loadPcd("../data/pcd/data_1/0000000006.pcd");
    cityBlock(viewer, pointProcessor, inputCloud);
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
    */
    
    while (!viewer->wasStopped ()){

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = pointProcessor->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloud);
        
        streamIterator++;

        if (streamIterator == pathStream.end()){
            streamIterator = pathStream.begin();
            //sleep(1);
        }
        
        viewer->spinOnce ();
        //sleep(0.5);
    } 
    
}