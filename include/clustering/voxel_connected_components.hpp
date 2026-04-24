#pragma once

#include "clustering/clusterer_interface.hpp"
#include <unordered_map>
#include <vector>

namespace lidar_perception {

/**
 * @struct VoxelKey
 * @brief Represents the 3D integer coordinates of a voxel in a grid.
 */
struct VoxelKey {
    int x, y, z;

    /**
     * @brief Equality operator for VoxelKey comparison.
     * @param other VoxelKey to compare with.
     * @return true if all coordinates are equal, false otherwise.
     */
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

/**
 * @struct VoxelKeyHasher
 * @brief Custom hash function for VoxelKey to be used in std::unordered_map.
 */
struct VoxelKeyHasher {
    /**
     * @brief Hashes the provided VoxelKey coordinate.
     * @param k VoxelKey to hash.
     * @return Generated hash value.
     */
    std::size_t operator()(const VoxelKey& k) const {
        return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
    }
};

/**
 * @class VoxelConnectedComponents
 * @brief Performs 3D Connected Components clustering among voxels.
 * 
 * Divides the point cloud into a 3D grid and groups points from neighboring voxels 
 * (26-connectivity) together. This provides a robust balance between speed and 3D precision.
 */
class VoxelConnectedComponents : public ClustererInterface {
public:
    /**
     * @brief Constructor for VoxelConnectedComponents.
     * @param voxel_size Size of each cubic voxel (meters).
     * @param min_cluster_size Minimum points for a valid cluster.
     * @param max_cluster_size Maximum points for a valid cluster.
     */
    VoxelConnectedComponents(float voxel_size = 0.15f, int min_cluster_size = 3, int max_cluster_size = 500);

    /**
     * @brief Default destructor.
     */
    ~VoxelConnectedComponents() override = default;

    /**
     * @brief Clusters the input cloud using a 3D grid and connected components.
     * @param cloud Input obstacle point cloud.
     * @param clusters Identified clusters.
     */
    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float voxel_size_;        ///< Voxel edge length.
    float inv_voxel_size_;    ///< Inverse of voxel size (precomputed for speed).
    int min_cluster_size_;    ///< Minimum size filter for clusters.
    int max_cluster_size_;    ///< Maximum size filter for clusters.
};

} // namespace lidar_perception
