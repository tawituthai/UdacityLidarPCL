/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

    // Time Ransac process
    auto startTime = std::chrono::steady_clock::now();
	// For max iterations
	while (maxIterations--)
	{
		// Randomly pick two points from cloud
		std::unordered_set<int> inliers;
		while (inliers.size() < 2) {
			inliers.insert(rand()%cloud->points.size());
		}

		// Get position of both points
		float x1,y1,x2,y2;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;	// move pointer to next element
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		// Create a line from points:
		// https://brilliant.org/wiki/dot-product-distance-between-point-and-a-line/
		float A = (y1 - y2);
		float B = (x2 - x1);
		float C = (x1*y2) - (x2*y1);
		// Go through all points in cloud
		for (int index = 0; index < cloud->points.size(); index++) {
			// make sure we ignore the 2 points we randomly pick to create line
			if (inliers.count(index) > 0) {
				continue;	// skip current for-loop
			}
			pcl::PointXYZ point_ = cloud->points[index];
			// Measure distance between every point and fitted line
			float Dist_ = fabs(A*point_.x + B*point_.y + C)/sqrt(pow(A,2) + pow(B,2));
			// If distance is smaller than threshold count it as inlier
			if (Dist_ <= distanceTol) {
				inliers.insert(index);
			}
		}
		// Check if the current iteration have got more inlier than the previous one
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));	// Initialize random number generator
	// Time RansacPlane process
    auto startTime = std::chrono::steady_clock::now();
	while (maxIterations--) {
		std::unordered_set<int> inliers_ind;
		while (inliers_ind.size() < 3) {
			inliers_ind.insert(rand()%cloud->points.size());
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
		float cross_A = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
		float cross_B = ((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1));
		float cross_C = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1));
		float cross_D = -1*((cross_A*x1) + (cross_B*y1) + (cross_C*z1));

		// Go through all points in cloud
		for (int index = 0; index < cloud->points.size(); index++) {
			// make sure we ignore the 3 points we randomly pick to create line
			if (inliers_ind.count(index) > 0) {
				continue;	// skip current for-loop
			}
			pcl::PointXYZ point_ = cloud->points[index];
			// Measure distance between every point and fitted line
			float Dist_ = fabs(cross_A*point_.x + cross_B*point_.y + cross_C*point_.z + cross_D)/sqrt(pow(cross_A,2) + pow(cross_B,2) + + pow(cross_C,2));
			// If distance is smaller than threshold count it as inlier
			if (Dist_ <= distanceTol) {
				inliers_ind.insert(index);
			}
		}
		// Check if the current iteration have got more inlier than the previous one
		if (inliers_ind.size() > inliersResult.size()) {
			inliersResult = inliers_ind;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RansacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;
	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());		// Plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());	// Obstacles
	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
