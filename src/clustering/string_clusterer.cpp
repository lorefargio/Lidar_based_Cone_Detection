#include "clustering/string_clusterer.hpp"
#include <cmath>

namespace fs_perception {

void StringClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) {
        return;
    }

    // Parameters optimized for high-density sensors (e.g., Hesai Pandar40P)
    const float MAX_DIST_ = 0.3f;        ///< Base distance threshold between consecutive points.
    const int MIN_CLUSTER_SIZE = 3;      ///< Minimum point count for a cone.
    const int MAX_CLUSTER_SIZE = 300;    ///< Maximum point count filter.
    const float MAX_INT_JUMP = 100.0f;   ///< Intensity jump limit between same object points.

    // Algorithm assumes points are ordered by ring and firing sequence by the driver.
    // This allows for linear O(N) clustering by checking point-to-point proximity.

    PointCloudPtr current_cluster(new PointCloud);
    current_cluster->push_back(cloud->points[0]);

    for (size_t i = 1; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        const auto& prev = cloud->points[i-1];

        // Dynamic distance threshold scaling with radial range
        float range = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        float dynamic_threshold = MAX_DIST_ + (range * 0.02f);
        float max_dist_sq = dynamic_threshold * dynamic_threshold;

        // Squared Euclidean distance for performance
        float dist_sq = (pt.x - prev.x)*(pt.x - prev.x) + 
                        (pt.y - prev.y)*(pt.y - prev.y) + 
                        (pt.z - prev.z)*(pt.z - prev.z);

        // Check for intensity discontinuities indicative of object boundaries
        float int_diff = std::abs(pt.intensity - prev.intensity);
        bool intensity_jump = int_diff > MAX_INT_JUMP;
        
        // Connectivity check
        if (dist_sq < max_dist_sq && !intensity_jump) {
            current_cluster->push_back(pt);
        } else {
            // End of current string; save if size constraints are met
            if (current_cluster->size() >= MIN_CLUSTER_SIZE && current_cluster->size() <= MAX_CLUSTER_SIZE) {
                clusters.push_back(current_cluster);
            }
            // Start a new candidate cluster
            current_cluster.reset(new PointCloud);
            current_cluster->push_back(pt);
        }
    }

    // Final cluster validation after loop
    if (current_cluster->size() >= MIN_CLUSTER_SIZE) {
        clusters.push_back(current_cluster);
    }
}

} // namespace fs_perception