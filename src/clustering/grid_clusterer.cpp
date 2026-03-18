#include "clustering/grid_clusterer.hpp"
#include <queue>
#include <cmath>

namespace fs_perception {

GridClusterer::GridClusterer(float grid_resolution, float max_range, int min_cluster_size, int max_cluster_size)
    : grid_res_(grid_resolution), max_range_(max_range), min_size_(min_cluster_size), max_size_(max_cluster_size) {}

void GridClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) {
        return;
    }

    // Initialize a sparse 2D grid represented as an unordered map
    std::unordered_map<std::pair<int, int>, Cell, pair_hash> grid;

    // Phase 1: Voxelization / Grid Population
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        
        // Fast range check for processing efficiency
        if (std::abs(pt.x) > max_range_ || std::abs(pt.y) > max_range_) continue;

        // Map point to grid coordinates based on resolution
        int grid_x = static_cast<int>(std::floor(pt.x / grid_res_));
        int grid_y = static_cast<int>(std::floor(pt.y / grid_res_));
        
        grid[{grid_x, grid_y}].point_indices.push_back(i);
    }

    int current_cluster_id = 1;

    // BFS neighbor offsets (8-connectivity in 2D)
    const int dx[] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1, 1, -1, 0, 1};

    // Phase 2: Connected Components Analysis using BFS
    for (auto& [coord, cell] : grid) {
        // Skip cells already part of a cluster
        if (cell.id != -1) continue;

        PointCloudPtr current_cluster(new PointCloud);
        std::queue<std::pair<int, int>> q;
        
        // Initialize a new cluster
        cell.id = current_cluster_id;
        q.push(coord);

        while (!q.empty()) {
            auto curr = q.front();
            q.pop();

            // Add all points belonging to this cell to the current cluster
            for (int idx : grid[curr].point_indices) {
                current_cluster->push_back(cloud->points[idx]);
            }

            // Check 8 neighboring cells for connectivity
            for (int i = 0; i < 8; ++i) {
                std::pair<int, int> neighbor = {curr.first + dx[i], curr.second + dy[i]};
                
                auto it = grid.find(neighbor);
                if (it != grid.end() && it->second.id == -1) {
                    it->second.id = current_cluster_id; // Mark as visited and assigned
                    q.push(neighbor);
                }
            }
        }

        // Apply size filters to validate the candidate object
        if (current_cluster->size() >= (size_t)min_size_ && current_cluster->size() <= (size_t)max_size_) {
            clusters.push_back(current_cluster);
        }
        
        current_cluster_id++;
    }
}

} // namespace fs_perception
