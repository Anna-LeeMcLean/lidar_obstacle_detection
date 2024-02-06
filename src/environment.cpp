/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "sensors/lidar.h"
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
    renderPointCloud(viewer, cloudFiltered, "filteredCloud");
}

/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = lidar->scan();
    //renderRays(viewer, lidar->position, point_cloud);
    //renderPointCloud(viewer, point_cloud, "point_cloud", Color(1.0, 0.0, 0.0));

    // TODO:: Create point processor
    int maxIterations = 100;
    float distanceThreshold = 0.2;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> point_cloud_pair;

    ProcessPointClouds<pcl::PointXYZ>* ppc = new ProcessPointClouds<pcl::PointXYZ>();
    point_cloud_pair = ppc->SegmentPlane(point_cloud, maxIterations, distanceThreshold);

    // Render plane and obstacle point clouds
    renderPointCloud(viewer, point_cloud_pair.first, "plane_point_cloud", Color(0.0, 1.0, 0.0));
    //renderPointCloud(viewer, point_cloud_pair.second, "obstacle_point_cloud", Color(1.0, 0.0, 0.0));

    // Create clusters within the obstacle pont cloud to distinguish the individual obstacles (cars)
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> distinct_obstacle_point_clouds = ppc->Clustering(point_cloud_pair.second, 1.5, 3, 100);

    int cluster_id = 0;
    std::vector<Color> colors {Color(1,0,0), Color(1,1,0), Color(1,0,1)};

    for (const auto cluster : distinct_obstacle_point_clouds){
        std::cout << "cluster size: ";
        ppc->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacle"+std::to_string(cluster_id), colors[cluster_id]);
        // Add bounding boxes around each of the distinct obstacle point clouds
        Box clusterBox = ppc->BoundingBox(cluster);
        renderBox(viewer, clusterBox, cluster_id, colors[cluster_id], 0.5);
        //renderBox(viewer, clusterBox, cluster_id);
        ++cluster_id;
    }

    
}
*/

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