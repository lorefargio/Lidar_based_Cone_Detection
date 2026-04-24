#include "clustering/voxel_connected_components.hpp"
#include <queue>

namespace lidar_perception {

VoxelConnectedComponents::VoxelConnectedComponents(float voxel_size, int min_cluster_size, int max_cluster_size)
    : voxel_size_(voxel_size), inv_voxel_size_(1.0f / voxel_size), 
      min_cluster_size_(min_cluster_size), max_cluster_size_(max_cluster_size) {}

void VoxelConnectedComponents::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) {
        return;
    }

    // Initialize a 3D sparse voxel grid mapping each coordinate to point indices
    std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHasher> voxel_grid;

    // Phase 1: Voxelization (O(N))
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];
        VoxelKey k;
        k.x = static_cast<int>(std::floor(pt.x * inv_voxel_size_));
        k.y = static_cast<int>(std::floor(pt.y * inv_voxel_size_));
        k.z = static_cast<int>(std::floor(pt.z * inv_voxel_size_));
        voxel_grid[k].push_back(i);
    }

    // Tracker for visited voxels during connectivity search
    std::unordered_map<VoxelKey, bool, VoxelKeyHasher> visited;
    for (const auto& pair : voxel_grid) {
        visited[pair.first] = false;
    }

    // Phase 2: 3D Connected Components using 26-connectivity (O(V))
    // Connectivity offsets for X, Y, Z directions
    int dx[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1};
    int dy[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1};
    int dz[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    for (const auto& pair : voxel_grid) {
        // Skip already processed voxels
        if (visited[pair.first]) continue;

        PointCloudPtr current_cluster(new PointCloud);
        std::queue<VoxelKey> queue;
        
        // Start a new cluster with the current voxel
        visited[pair.first] = true;
        queue.push(pair.first);

        while (!queue.empty()) {
            VoxelKey curr = queue.front();
            queue.pop();

            // Add all points falling in this voxel to the cluster
            for (int idx : voxel_grid[curr]) {
                current_cluster->push_back(cloud->points[idx]);
            }

            // Expand to all 26 neighboring voxels
            for (int i = 0; i < 27; ++i) {
                // Ignore self-comparison
                if (dx[i] == 0 && dy[i] == 0 && dz[i] == 0) continue;

                VoxelKey neighbor = {curr.x + dx[i], curr.y + dy[i], curr.z + dz[i]};
                
                auto it = voxel_grid.find(neighbor);
                if (it != voxel_grid.end() && !visited[neighbor]) {
                    visited[neighbor] = true; // Mark as visited and added to queue
                    queue.push(neighbor);
                }
            }
        }

        // Filter valid clusters based on point count
        if (current_cluster->size() >= (size_t)min_cluster_size_ && current_cluster->size() <= (size_t)max_cluster_size_) {
            clusters.push_back(current_cluster);
        }
    }
}

} // namespace lidar_perception
