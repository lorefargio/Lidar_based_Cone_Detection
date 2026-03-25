#include "clustering/string_clusterer.hpp"
#include <cmath>

namespace fs_perception {

StringClusterer::StringClusterer() : config_(Config()) {}

StringClusterer::StringClusterer(const Config& config) : config_(config) {}

void StringClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) {
        return;
    }

    // Algorithm assumes points are ordered by ring and firing sequence by the driver.
    // This allows for linear O(N) clustering by checking point-to-point proximity.

    PointCloudPtr current_cluster(new PointCloud);
    current_cluster->push_back(cloud->points[0]);

    for (size_t i = 1; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        const auto& prev = cloud->points[i-1];

        // Dynamic distance threshold scaling with radial range
        float range = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        float dynamic_threshold = config_.max_dist + (range * 0.02f);
        float max_dist_sq = dynamic_threshold * dynamic_threshold;

        // Squared Euclidean distance for performance
        float dist_sq = (pt.x - prev.x)*(pt.x - prev.x) + 
                        (pt.y - prev.y)*(pt.y - prev.y) + 
                        (pt.z - prev.z)*(pt.z - prev.z);

        // Check for intensity discontinuities indicative of object boundaries
        float int_diff = std::abs(pt.intensity - prev.intensity);
        bool intensity_jump = int_diff > config_.max_int_jump;
        
        // Connectivity check
        if (dist_sq < max_dist_sq && !intensity_jump) {
            current_cluster->push_back(pt);
        } else {
            // End of current string; save if size constraints are met
            if (current_cluster->size() >= (size_t)config_.min_cluster_size && current_cluster->size() <= (size_t)config_.max_cluster_size) {
                clusters.push_back(current_cluster);
            }
            // Start a new candidate cluster
            current_cluster.reset(new PointCloud);
            current_cluster->push_back(pt);
        }
    }

    // Final cluster validation after loop
    if (current_cluster->size() >= (size_t)config_.min_cluster_size) {
        clusters.push_back(current_cluster);
    }
}

} // namespace fs_perception