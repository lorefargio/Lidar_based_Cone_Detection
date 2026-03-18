#pragma once

#include "utils/types.hpp"
#include <vector>

namespace fs_perception {

/**
 * @class ClustererInterface
 * @brief Common interface for all clustering algorithms.
 * 
 * Defines the standard structure for raggruping obstacle points into distinct object clusters.
 */
class ClustererInterface {
public:
    virtual ~ClustererInterface() = default;

    /**
     * @brief Groups obstacle points into distinct clusters.
     * @param cloud[in] Point cloud containing obstacle points only (ground removed).
     * @param clusters[out] Vector of point clouds, where each cloud represents a single object cluster.
     */
    virtual void cluster(const PointCloudPtr& cloud, 
                         std::vector<PointCloudPtr>& clusters) = 0;
};

} // namespace fs_perception
