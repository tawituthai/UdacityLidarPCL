/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include <unordered_set>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");
    Car car4(Vect3(-6, -2, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car4");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);
    cars.push_back(car4);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
        car4.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position , cloud);
    renderPointCloud(viewer, inputCloud, "lidar", Color(0, 1.0, 1.0));
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // Time PCL's SegmentPlane process
    auto startTime = std::chrono::steady_clock::now();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 1000, 0.2);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "PCL SegmentPlane took: " << elapsedTime.count() << " milliseconds" << std::endl;

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 2.0, 4, 100);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 1), Color(1, 1, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
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
            pcl::PointXYZI point_ = cloud->points[index];
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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;
    return inliersResult;
}

void customPipeline(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // Downsampling point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr fliteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));
    // Segment point cloud, separate between obstacles and road
    std::unordered_set<int> inliers = RansacPlane(fliteredCloud, 25, 0.3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());  // Plane
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>()); // Obstacles
    for (int index = 0; index < fliteredCloud->points.size(); index++)
    {
        pcl::PointXYZI point = fliteredCloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);  // Plane (Green)
        else
            cloudOutliers->points.push_back(point); // Obstacles (Red)
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size())
    {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    }
    else
    {
        renderPointCloud(viewer, fliteredCloud, "data");
    }
}

// This is for PCD stream
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // Downsampling point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr fliteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));

    // Segment point cloud, separate between obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(fliteredCloud, 25, 0.3);
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // Clustering, only cluster an obstacle
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 1), Color(1, 1, 1),
                                 Color(1, 1, 0.5), Color(1, 0.5, 1), Color(1, 0.5, 0.5)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

// This is for single PCD file
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // Import point cloud with intensity
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // Downsampling point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr fliteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-20, -10, -2, 1), Eigen::Vector4f(20, 10, 10, 1));
    // Segment point cloud, separate between obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(fliteredCloud, 1000, 0.2);
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    // Clustering, only cluster an obstacle
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 1.0, 4, 400);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 1), Color(1, 1, 1),
                                 Color(1, 1, 0.5), Color(1, 0.5, 1), Color(1, 0.5, 0.5)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        //cityBlock(viewer, pointProcessorI, inputCloudI);
        customPipeline(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}