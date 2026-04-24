#pragma once

#include "clustering/clusterer_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace lidar_perception {

/**
 * @class DBSCANClusterer
 * @brief Implementation of the Density-Based Spatial Clustering of Applications with Noise (DBSCAN) algorithm.
 * 
 * This implementation optimizes the traditional DBSCAN approach by replacing the $O(N \log N)$ KD-Tree 
 * radius search with a "Hash-Killer" Pre-allocated 3D Flat Grid. The world is discretized into a 
 * 1D array representing a 3D volume, reducing neighbor lookup to $O(1)$ amortized complexity.
 * 
 * @note This optimization ensures deterministic execution times and is cache-friendly, making it 
 * suitable for high-frequency real-time perception in racing environments.
 */
class DBSCANClusterer : public ClustererInterface {
public:
    /**
     * @brief Constructs a DBSCANClusterer with specified hyperparameters.
     * @param eps The maximum distance $\epsilon$ between two samples for neighborhood inclusion.
     * @param min_pts The minimum number of samples required in a neighborhood to define a core point.
     * @param min_cluster_size Minimum cardinality filter for valid clusters.
     * @param max_cluster_size Maximum cardinality filter to exclude oversized or merged clusters.
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

} // namespace lidar_perception