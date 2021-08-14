// PCL lib Functions for processing point clouds
#include <unordered_set>
#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    // Set region of interest
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true); // Set to true if you want to be able to extract the indices of points being removed
    region.setInputCloud(cloud_filtered);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.filter(*cloud_region);

    // Filter out point from roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true); // Set to true if you want to be able to extract the indices of points being removed
    region.setInputCloud(cloud_region);
    region.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    region.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region.filter(indices);

    // Get points that's inside the box
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }
    // Extract (remove) point that's in the box from cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true); // discard the inliers indice, keep to outliers
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took: " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());

    // Set inliers into planeCloud
    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Create extractor
    pcl::ExtractIndices<PointT> extract;
    // Extract outliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Extract everythings that's not inliers
    extract.filter(*obstCloud); // Keep extraction result in ObstCloud (non-road points)

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Do segmentation find plane from cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Check segmentation output
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took: " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());  // Plane
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>()); // Obstacles

    srand(time(NULL)); // Initialize random number generator
                       // Time RansacPlane process
    auto startTime = std::chrono::steady_clock::now();
    while (maxIterations--)
    {
        std::unordered_set<int> inliers_ind;
        while (inliers_ind.size() < 3)
        {
            inliers_ind.insert(rand() % cloud->points.size());
        }

        // Get a position of each points
        float x1, y1, z1;
        float x2, y2, z2;
        float x3, y3, z3;
        auto itr = inliers_ind.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        // Calculate cross product
        float cross_A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
        float cross_B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
        float cross_C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
        float cross_D = -1 * ((cross_A * x1) + (cross_B * y1) + (cross_C * z1));

        // Go through all points in cloud
        for (int index = 0; index < cloud->points.size(); index++)
        {
            // make sure we ignore the 3 points we randomly pick to create line
            if (inliers_ind.count(index) > 0)
            {
                continue; // skip current for-loop
            }
            PointT point_ = cloud->points[index];
            // Measure distance between every point and fitted line
            float Dist_ = fabs(cross_A * point_.x + cross_B * point_.y + cross_C * point_.z + cross_D) / sqrt(pow(cross_A, 2) + pow(cross_B, 2) + +pow(cross_C, 2));
            // If distance is smaller than threshold count it as inlier
            if (Dist_ <= distanceTol)
            {
                inliers_ind.insert(index);
            }
        }
        // Check if the current iteration have got more inlier than the previous one
        if (inliers_ind.size() > inliersResult.size())
        {
            inliersResult = inliers_ind;
        }
    }

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);  // Plane (Green)
        else
            cloudOutliers->points.push_back(point); // Obstacles (Red)
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> EC_cluster;
    EC_cluster.setClusterTolerance(clusterTolerance); // 2cm
    EC_cluster.setMinClusterSize(minSize);            // 100
    EC_cluster.setMaxClusterSize(maxSize);            // 25000
    EC_cluster.setSearchMethod(tree);
    EC_cluster.setInputCloud(cloud);
    EC_cluster.extract(cluster_indices);

    for (pcl::PointIndices getIndices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto idx : getIndices.indices)
        {
            cloud_cluster->points.push_back(cloud->points[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}