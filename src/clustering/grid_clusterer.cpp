#include "clustering/grid_clusterer.hpp"
#include <queue>
#include <cmath>

namespace fs_perception {

GridClusterer::GridClusterer(float grid_resolution, float max_range, int min_cluster_size, int max_cluster_size)
    : grid_res_(grid_resolution), max_range_(max_range), min_size_(min_cluster_size), max_size_(max_cluster_size) {
    grid_dim_ = static_cast<int>(std::ceil(2.0f * max_range_ / grid_res_)) + 2;
    grid_.resize(grid_dim_ * grid_dim_);
}

void GridClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) {
        return;
    }

    // Keep track of which cells are occupied this frame to avoid full grid scanning
    std::vector<int> occupied_indices;
    occupied_indices.reserve(cloud->size());

    // Phase 1: Voxelization / Grid Population
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        
        if (std::abs(pt.x) > max_range_ || std::abs(pt.y) > max_range_) continue;

        int gx = static_cast<int>(std::floor(pt.x / grid_res_));
        int gy = static_cast<int>(std::floor(pt.y / grid_res_));
        
        int idx = getGridIndex(gx, gy);
        if (idx < 0 || idx >= static_cast<int>(grid_.size())) continue;

        if (grid_[idx].point_indices.empty()) {
            occupied_indices.push_back(idx);
            grid_[idx].id = -1; // Reset for this frame
        }
        grid_[idx].point_indices.push_back(i);
    }

    int current_cluster_id = 1;

    // BFS neighbor offsets (8-connectivity in 2D)
    const int dx[] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1, 1, -1, 0, 1};

    // Phase 2: Connected Components Analysis using BFS
    for (int start_idx : occupied_indices) {
        Cell& start_cell = grid_[start_idx];
        if (start_cell.id != -1) continue;

        PointCloudPtr current_cluster(new PointCloud);
        std::queue<int> q;
        
        start_cell.id = current_cluster_id;
        q.push(start_idx);

        while (!q.empty()) {
            int curr_idx = q.front();
            q.pop();

            for (int p_idx : grid_[curr_idx].point_indices) {
                current_cluster->push_back(cloud->points[p_idx]);
            }

            // Reconstruct coordinates from flat index
            int gx = curr_idx / grid_dim_ - grid_dim_ / 2;
            int gy = curr_idx % grid_dim_ - grid_dim_ / 2;

            for (int i = 0; i < 8; ++i) {
                int nx = gx + dx[i];
                int ny = gy + dy[i];
                int n_idx = getGridIndex(nx, ny);

                if (n_idx >= 0 && n_idx < static_cast<int>(grid_.size())) {
                    Cell& neighbor = grid_[n_idx];
                    if (!neighbor.point_indices.empty() && neighbor.id == -1) {
                        neighbor.id = current_cluster_id;
                        q.push(n_idx);
                    }
                }
            }
        }

        if (current_cluster->size() >= (size_t)min_size_ && current_cluster->size() <= (size_t)max_size_) {
            clusters.push_back(current_cluster);
        }
        
        current_cluster_id++;
    }

    // Phase 3: Cleanup only occupied cells for next frame
    for (int idx : occupied_indices) {
        grid_[idx].point_indices.clear();
        grid_[idx].id = -1;
    }
}

} // namespace fs_perception
