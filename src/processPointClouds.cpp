// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Voxel grid filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);

    std::cout << "Input point cloud size: " << cloud->width * cloud->height << std::endl;
    std::cout << "Point cloud size after downsampling: " << cloudFiltered->width * cloudFiltered->height << std::endl;
    
    // Remove points in the point cloud beyond max and min points. 
    typename pcl::PointCloud<PointT>::Ptr croppedFilteredCloud (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> croppedCloudBox (true);
    croppedCloudBox.setMin(minPoint);
    croppedCloudBox.setMax(maxPoint);
    croppedCloudBox.setInputCloud(cloudFiltered);
    croppedCloudBox.filter(*croppedFilteredCloud);

    std::cout << "Point cloud size after cropping to region of interest: " << croppedFilteredCloud->width * croppedFilteredCloud->height << std::endl;
    
    // Remove points which represent the roof of the car
    typename pcl::PointCloud<PointT>::Ptr regionOfInterestCloud (new pcl::PointCloud<PointT>);
    std::vector<int> roofIndicesVec;
    pcl::CropBox<PointT> roofPoints (true);
    roofPoints.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    roofPoints.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    roofPoints.setInputCloud(croppedFilteredCloud);
    roofPoints.filter(roofIndicesVec);

    typename pcl::PointIndices::Ptr roofIndices (new pcl::PointIndices);
    for (auto index : roofIndicesVec){
        roofIndices->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(croppedFilteredCloud);
    extractor.setIndices(roofIndices);
    extractor.setNegative(true);
    extractor.filter(*regionOfInterestCloud);

    std::cout << "Point cloud size after removing roof points: " << regionOfInterestCloud->width * regionOfInterestCloud->height << std::endl;
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    

    return regionOfInterestCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extractor;
    typename pcl::PointCloud<PointT>::Ptr plane_point_cloud {new pcl::PointCloud<PointT>()};    // New point cloud to store the plane points in.
    typename pcl::PointCloud<PointT>::Ptr obstacle_point_cloud {new pcl::PointCloud<PointT>()};    // New point cloud to store the obstacle points in.

    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);
    extractor.setNegative(false);           // I think when the negative is set to false, the inliers are stored in the cloud object passed to the filter function.
    extractor.filter(*plane_point_cloud);   // Stores the points which belong to the plane in the plane point cloud.

    extractor.setNegative(true);            // When it's true, everything that's not the inliers are stored in the cloud object passed to the filter function.
    extractor.filter(*obstacle_point_cloud);

    std::cout << "Input point cloud has: " << cloud->height * cloud->width << " data points." << std::endl;
    std::cout << "Plane point cloud has: " << plane_point_cloud->height * plane_point_cloud->width << " data points." << std::endl;
    std::cout << "Obstacle point cloud has: " << obstacle_point_cloud->height * obstacle_point_cloud->width << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(plane_point_cloud, obstacle_point_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    // create coefficient, inliers and segmenter objects.
    // INTERESTING NOTE: Two different ways to create pointers to an object on the heap. Either [* and =] OR [::Ptr and ()/{}].
    typename pcl::ModelCoefficients* model_coefficients = new pcl::ModelCoefficients();
    typename pcl::PointIndices::Ptr inliers (new pcl::PointIndices());       // will store the indices of the points in the point cloud which belong to the plane.
    pcl::SACSegmentation<PointT> segmenter;                  // will segment the plane from the other objects in the point cloud using RANSAC.
    

    // setting parameters for the segment
    segmenter.setOptimizeCoefficients(true);
    segmenter.setModelType(pcl::SACMODEL_PLANE);    // Segmenting a plane from the input cloud
    segmenter.setMethodType(pcl::SAC_RANSAC);        // Using the RANSAC algorithm
    segmenter.setDistanceThreshold(distanceThreshold);
    segmenter.setMaxIterations(maxIterations);

    // Segment the input cloud.
    segmenter.setInputCloud(cloud);
    segmenter.segment(*inliers, *model_coefficients);     // I think this stores the points belonging to the plane in the inliers object

    /// Make sure segmentation was successful by seeing if inliiers is poopulated with data
    if (inliers->indices.size() == 0){
        std::cerr << "Could not segment plane points from input cloud." << std::endl;
        // we need to return an empty pair or something..
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // We have the indices of the points belonging to the plane. Now we extract them from the original point cloud.
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Create KDTree oject for extraction search method
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;     // holds the indices for each detected cluster
    pcl::EuclideanClusterExtraction<PointT> ec_extractor;
    ec_extractor.setClusterTolerance(clusterTolerance);
    ec_extractor.setMinClusterSize(minSize);
    ec_extractor.setMaxClusterSize(maxSize);
    ec_extractor.setSearchMethod(tree);
    ec_extractor.setInputCloud(cloud);
    ec_extractor.extract(cluster_indices);      // this method stores an instance of PointIndices for each cluster detected. i.e cluster_indices[0] will hold the indices for the first cluster

    for (const auto& cluster : cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr pc (new pcl::PointCloud<PointT>);
        for (const auto index : cluster.indices){
            pc->points.push_back(cloud->points[index]);
        }
        pc->width = pc->size();
        pc->height = 1;
        pc->is_dense = true;
        clusters.push_back(pc);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}