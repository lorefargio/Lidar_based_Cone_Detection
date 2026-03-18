#pragma once

#include "clustering/clusterer_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace fs_perception {

/**
 * @brief Adaptive DBSCAN acts as a lightweight replacement for HDBSCAN.
 * It adjusts the epsilon radius dynamically based on the point's distance 
 * from the sensor, handling the variable density of LiDAR rings perfectly.
 */
class AdaptiveDBSCANClusterer : public ClustererInterface {
public:
    AdaptiveDBSCANClusterer(float eps_base = 0.15f, float alpha = 0.015f, int min_pts = 3, 
                            int min_cluster_size = 3, int max_cluster_size = 500);
    ~AdaptiveDBSCANClusterer() override = default;

    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float eps_base_;
    float alpha_;
    int min_pts_;
    int min_cluster_size_;
    int max_cluster_size_;
};

} // namespace fs_perception
