#pragma once

#include "clustering/clusterer_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace fs_perception {

/**
 * @class AdaptiveDBSCANClusterer
 * @brief Distance-Aware Density-Based Spatial Clustering of Applications with Noise (DBSCAN).
 * 
 * To compensate for LiDAR beam divergence and the resulting decrease in point density at high 
 * ranges, this implementation employs a dynamic parameter model. Epsilon scales linearly 
 * with radial distance $r$: $\epsilon(r) = \epsilon_{base} + \alpha r$.
 * 
 * Like the standard DBSCANClusterer, it utilizes a "Hash-Killer" Pre-allocated 3D Flat Grid 
 * to achieve $O(1)$ amortized neighbor lookup, ensuring real-time performance on high-resolution 
 * point clouds.
 */
class AdaptiveDBSCANClusterer : public ClustererInterface {
public:
    /**
     * @brief Constructs an AdaptiveDBSCANClusterer with distance-scaling hyperparameters.
     * @param eps_base Base epsilon radius $\epsilon_{base}$ at the sensor origin ($r=0$).
     * @param alpha Growth coefficient $\alpha$ governing the expansion of $\epsilon$ per meter of range.
     * @param min_pts Minimum neighbor count $\kappa$ required for core point classification.
     * @param min_cluster_size Minimum cardinality filter for valid clusters.
     * @param max_cluster_size Maximum cardinality filter to exclude merged or large-scale obstacles.
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