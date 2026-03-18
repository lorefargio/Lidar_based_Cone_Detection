#pragma once

#include "clustering/clusterer_interface.hpp"
#include <unordered_map>
#include <vector>

namespace fs_perception {

struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHasher {
    std::size_t operator()(const VoxelKey& k) const {
        // Simple fast hash for 3D coordinates
        return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
    }
};

class VoxelConnectedComponents : public ClustererInterface {
public:
    VoxelConnectedComponents(float voxel_size = 0.15f, int min_cluster_size = 3, int max_cluster_size = 500);
    ~VoxelConnectedComponents() override = default;

    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float voxel_size_;
    float inv_voxel_size_;
    int min_cluster_size_;
    int max_cluster_size_;
};

} // namespace fs_perception