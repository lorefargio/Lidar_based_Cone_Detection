#pragma once

#include "clustering/clusterer_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace fs_perception {

/**
 * @class AdaptiveDBSCANClusterer
 * @brief Lightweight replacement for HDBSCAN that adjusts epsilon based on distance.
 * 
 * This algorithm dynamically scales the DBSCAN epsilon radius based on the point's distance 
 * from the sensor, effectively handling the variable point density characteristic of LiDAR scans.
 */
class AdaptiveDBSCANClusterer : public ClustererInterface {
public:
    /**
     * @brief Constructor for AdaptiveDBSCANClusterer.
     * @param eps_base Base epsilon radius at 0m distance.
     * @param alpha Growth factor for epsilon based on radial distance.
     * @param min_pts Minimum points required to form a dense region (core point).
     * @param min_cluster_size Minimum points required to consider a cluster valid.
     * @param max_cluster_size Maximum points allowed in a cluster.
     */
    AdaptiveDBSCANClusterer(float eps_base = 0.15f, float alpha = 0.015f, int min_pts = 3, 
                            int min_cluster_size = 3, int max_cluster_size = 500);

    /**
     * @brief Default destructor.
     */
    ~AdaptiveDBSCANClusterer() override = default;

    /**
     * @brief Performs clustering on the provided point cloud.
     * @param cloud Input cloud containing obstacle points.
     * @param clusters Output vector to store identified clusters.
     */
    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float eps_base_;          ///< Base epsilon for distance-based scaling.
    float alpha_;             ///< Slope for epsilon radius growth.
    int min_pts_;             ///< Minimum neighbors within epsilon for core point.
    int min_cluster_size_;    ///< Minimum size filter for clusters.
    int max_cluster_size_;    ///< Maximum size filter for clusters.
};

} // namespace fs_perception