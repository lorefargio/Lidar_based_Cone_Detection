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
     * @struct Config
     * @brief Parameters for the string clustering algorithm.
     */
    struct Config {
        float max_dist = 0.3f;        ///< Base distance threshold between consecutive points.
        int min_cluster_size = 3;      ///< Minimum point count for a cone.
        int max_cluster_size = 300;    ///< Maximum point count filter.
        float max_int_jump = 100.0f;   ///< Intensity jump limit between same object points.
    };

    /**
     * @brief Default constructor for StringClusterer.
     */
    StringClusterer();

    /**
     * @brief Explicit constructor with custom configuration.
     * @param config The custom configuration struct.
     */
    explicit StringClusterer(const Config& config);

    /**
     * @brief Performs linear string clustering on the input point cloud.
     * @param cloud Input obstacle point cloud.
     * @param clusters Identified clusters.
     */
    void cluster(const PointCloudPtr& cloud, 
                 std::vector<PointCloudPtr>& clusters) override;

private:
    Config config_; ///< Algorithm configuration.
};

} // namespace fs_perception