// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

// Structure to represent node of kd tree
struct Node
{
    // std::vector<float> point;
    pcl::PointXYZI point;
    int id;
    Node *left;
    Node *right;

    Node(pcl::PointXYZI arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct KdTree
{
    Node *root;

    KdTree()
        : root(NULL)
    {
    }

    ~KdTree()
    {
        delete root;
    }

    void insertHelper(Node **node, uint depth, pcl::PointXYZI point, int id)
    {
        // Tree is empty
        if (*node == NULL)
        {
            *node = new Node(point, id);
        }
        else
        {
            // up to 3 dimentions
            uint cd = depth % 3;
            if (cd == 0)
            { // X
                if (point.x < ((*node)->point.x))
                {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                }
                else
                {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
            else if (cd == 1)
            { // Y
                if (point.y < ((*node)->point.y))
                {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                }
                else
                {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
            else if (cd == 2)
            { // Z
                if (point.z < ((*node)->point.z))
                {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                }
                else
                {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
        }
    }

    void insert(pcl::PointXYZI point, int id)
    {
        insertHelper(&root, 0, point, id);
    }

    void searchHelper(pcl::PointXYZI target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
    {
        if (node != NULL)
        {
            if ((node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) &&
                (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol)) &&
                (node->point.z >= (target.z - distanceTol)) && (node->point.z <= (target.z + distanceTol)))
            {
                // Calculate Euclidean distance
                float distance = sqrt(((node->point.x - target.x) * (node->point.x - target.x)) +
                                      ((node->point.y - target.y) * (node->point.y - target.y)) +
                                      ((node->point.z - target.z) * (node->point.z - target.z)));
                if (distance <= distanceTol)
                {
                    ids.push_back(node->id);
                }
            }

            // Check across boundary
            uint cd = depth % 3;
            if (cd == 0)
            { // X
                if ((target.x - distanceTol) < (node->point.x))
                {
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                }
                if ((target.x + distanceTol) >= (node->point.x))
                {
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                }
            }
            else if (cd == 1)
            { // Y
                if ((target.y - distanceTol) < (node->point.y))
                {
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                }
                if ((target.y + distanceTol) >= (node->point.y))
                {
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                }
            }
            else if (cd == 2)
            { // Z
                if ((target.z - distanceTol) < (node->point.z))
                {
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                }
                if ((target.z + distanceTol) >= (node->point.z))
                {
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
                }
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(pcl::PointXYZI target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;
    }
};

template <typename PointT>
class ProcessPointClouds
{
public:
    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster_ids, std::vector<bool>& processed, KdTree* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol, float minSize, float maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */