#pragma once

#include "utils/types.hpp"

namespace fs_perception {

/**
 * @class GroundRemoverInterface
 * @brief Common interface for all ground removal algorithms.
 * 
 * Defines the standard structure for separating ground points from obstacle points, 
 * which is the first and most critical stage of the perception pipeline.
 */
class GroundRemoverInterface {
public:
    virtual ~GroundRemoverInterface() = default;

    /**
     * @brief Separates the ground points from the input cloud.
     * @param cloud_in[in] Input point cloud (filtered for NaNs and range).
     * @param cloud_obstacles[out] Output cloud containing obstacle candidates.
     * @param cloud_ground[out] Output cloud containing ground points for debug visualization.
     */
    virtual void removeGround(const PointCloudConstPtr& cloud_in, 
                              PointCloudPtr& cloud_obstacles, 
                              PointCloudPtr& cloud_ground) = 0;
};

} // namespace fs_perception