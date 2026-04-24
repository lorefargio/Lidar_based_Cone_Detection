#pragma once

#include "clustering/clusterer_interface.hpp"

namespace lidar_perception {

/**
 * @class EuclideanClusterer
 * @brief Simple Euclidean distance-based clustering algorithm using a KD-Tree.
 * 
 * It groups points that are within a certain tolerance from each other. 
 * This is the standard PCL implementation wrapper.
 */
class EuclideanClusterer : public ClustererInterface {
public:
    /**
     * @brief Constructor for EuclideanClusterer.
     * @param cluster_tolerance The maximum distance between any two points in the same cluster.
     * @param min_cluster_size The minimum number of points that a cluster must contain.
     * @param max_cluster_size The maximum number of points that a cluster can contain.
     */
    EuclideanClusterer(float cluster_tolerance = 0.35f, int min_cluster_size = 3, int max_cluster_size = 300);

    /**
     * @brief Clusters the input cloud using Euclidean distance.
     * @param cloud[in] Input obstacle point cloud.
     * @param clusters[out] Identified clusters.
     */
    void cluster(const PointCloudPtr& cloud, 
                 std::vector<PointCloudPtr>& clusters) override;

private:
    float cluster_tolerance_; ///< Distance tolerance for raggruping.
    int min_cluster_size_;    ///< Minimum size filter.
    int max_cluster_size_;    ///< Maximum size filter.
};

} // namespace lidar_perception
