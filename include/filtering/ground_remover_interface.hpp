/**
 * @file ground_remover_interface.hpp
 * @brief Abstract base class for terrain segmentation algorithms.
 * 
 * Provides a standardized interface for various ground removal strategies. 
 * Efficient terrain segmentation is a foundational stage in the LiDAR perception 
 * pipeline, enabling the isolation of potential obstacle clusters.
 */

#pragma once

#include "utils/types.hpp"

namespace lidar_perception {

/**
 * @class GroundRemoverInterface
 * @brief Abstract interface defining the terrain segmentation protocol.
 * 
 * This interface establishes the contractual requirements for all ground removal 
 * implementations, ensuring modularity and interoperability within the perception stack.
 */
class GroundRemoverInterface {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived implementations.
     */
    virtual ~GroundRemoverInterface() = default;

    /**
     * @brief Configures an optional voxel grid filter for the resulting obstacle cloud.
     * 
     * @param leaf_size The isotropic voxel dimension (meters). A value of 0.0 disables filtering.
     */
    void setVoxelFilter(float leaf_size) { voxel_size_ = leaf_size; }

    /**
     * @brief Performs terrain segmentation on the input point cloud.
     * 
     * Divides the input points into two disjoint sets: points belonging to the 
     * traversable ground surface and points belonging to non-ground obstacles.
     * 
     * @param[in]  cloud_in The pre-processed input LiDAR point cloud.
     * @param[out] cloud_obstacles The resulting subset of non-ground obstacle points.
     * @param[out] cloud_ground The resulting subset of terrain points (primarily for diagnostic purposes).
     */
    virtual void removeGround(const PointCloudConstPtr& cloud_in, 
                              PointCloudPtr& cloud_obstacles, 
                              PointCloudPtr& cloud_ground) = 0;

protected:
    float voxel_size_ = 0.0f; ///< Dimension of the voxel filter; 0.0 indicates a disabled state.
};

} // namespace lidar_perception