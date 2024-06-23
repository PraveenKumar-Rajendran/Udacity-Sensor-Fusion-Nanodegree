#include <chrono>
#include <string>
#include "kdtree_3d.h"

template<typename PointT>
class PointCloudClusterer {
private:
    int totalPoints;           // Total number of points in the cloud
    float clusterTolerance;    // Distance tolerance for clustering
    int minClusterSize;        // Minimum size for a cluster to be valid
    int maxClusterSize;        // Maximum size for a cluster to be valid
    std::vector<bool> processedPoints; // Tracks whether each point has been processed
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // Stores the resulting clusters

public:
    PointCloudClusterer(int totalPts, float clusterTol, int minSize, int maxSize) :
        totalPoints(totalPts), clusterTolerance(clusterTol), minClusterSize(minSize), maxClusterSize(maxSize) {
            processedPoints.assign(totalPoints, false);
        }
    ~PointCloudClusterer();

    void proximity(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, std::vector<int>& cluster, int index);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclidCluster(typename pcl::PointCloud<PointT>::Ptr cloud);
};

template<typename PointT>
PointCloudClusterer<PointT>::~PointCloudClusterer() {} // Destructor

// Proximity function to add nearby points to the current cluster
template<typename PointT>
void PointCloudClusterer<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, std::vector<int>& cluster, int index) {
    processedPoints[index] = true; // Mark the point as processed
    cluster.push_back(index); // Add the point to the current cluster

    // Find all points within the cluster tolerance
    std::vector<int> nearbyPoints = tree->search(cloud->points[index], clusterTolerance);
    for(int pointIndex: nearbyPoints) {
        // Recursively process each nearby point
        if (!processedPoints[pointIndex]) {
            proximity(cloud, tree, cluster, pointIndex);
        }
    }
}

// Main function to perform Euclidean clustering on the point cloud
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> PointCloudClusterer<PointT>::EuclidCluster(typename pcl::PointCloud<PointT>::Ptr cloud) {
    KdTree* tree = new KdTree;

    // Insert points into the KD-Tree for efficient search
    for (int i = 0; i < totalPoints; i++)
        tree->insert(cloud->points[i], i);

    for (int i = 0; i < totalPoints; i++) {
        if (processedPoints[i]) {
            continue; // Skip already processed points
        }

        std::vector<int> clusterIndices;
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        proximity(cloud, tree, clusterIndices, i); // Find points close to the current point

        // Check if the cluster meets the size criteria
        int clusterSize = clusterIndices.size();
        if (clusterSize >= minClusterSize && clusterSize <= maxClusterSize) {
            // Add all points in the cluster to the cloud cluster
            for (int pointIndex = 0; pointIndex < clusterSize; pointIndex++) {
                cloudCluster->points.push_back(cloud->points[clusterIndices[pointIndex]]);
            }

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;

            clusters.push_back(cloudCluster); // Add the valid cluster to the list
        }
    }

    return clusters; // Return the list of found clusters
}
