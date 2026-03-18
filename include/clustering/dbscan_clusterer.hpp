#pragma once

#include "clustering/clusterer_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace fs_perception {

/**
 * @class DBSCANClusterer
 * @brief Density-Based Spatial Clustering of Applications with Noise (DBSCAN).
 * 
 * Groups points together that are packed closely together (points with many nearby neighbors), 
 * marking as outliers points that lie alone in low-density regions (noise).
 */
class DBSCANClusterer : public ClustererInterface {
public:
    /**
     * @brief Constructor for DBSCANClusterer.
     * @param eps The maximum distance between two samples for one to be considered as in the neighborhood of the other.
     * @param min_pts The number of samples in a neighborhood for a point to be considered as a core point.
     * @param min_cluster_size Minimum points required to consider a cluster valid.
     * @param max_cluster_size Maximum points allowed in a cluster.
     */
    DBSCANClusterer(float eps = 0.30f, int min_pts = 3, int min_cluster_size = 3, int max_cluster_size = 500);

    /**
     * @brief Default destructor.
     */
    ~DBSCANClusterer() override = default;

    /**
     * @brief Performs clustering on the provided point cloud.
     * @param cloud Input cloud containing obstacle points.
     * @param clusters Output vector to store identified clusters.
     */
    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float eps_;               ///< Search radius for dense regions.
    int min_pts_;             ///< Minimum neighbors required for a core point.
    int min_cluster_size_;    ///< Minimum size filter for clusters.
    int max_cluster_size_;    ///< Maximum size filter for clusters.
};

} // namespace fs_perception