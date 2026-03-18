#pragma once

#include "utils/types.hpp"
#include "clustering/clusterer_interface.hpp"
#include <vector>

namespace fs_perception {

/**
 * @class StringClusterer
 * @brief Ultra-fast linear clustering that exploits point scan order.
 * 
 * This algorithm assumes the LiDAR driver publishes points ordered by ring and firing sequence. 
 * It groups consecutive points into "strings" if they are within a certain distance, 
 * avoiding the need for a KD-Tree (O(N) complexity).
 */
class StringClusterer : public ClustererInterface {
public:
    /**
     * @brief Performs linear string clustering on the input point cloud.
     * @param cloud Input obstacle point cloud.
     * @param clusters Identified clusters.
     */
    void cluster(const PointCloudPtr& cloud, 
                 std::vector<PointCloudPtr>& clusters) override;
};

} // namespace fs_perception