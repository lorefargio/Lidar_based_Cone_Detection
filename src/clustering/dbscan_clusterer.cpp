#include "clustering/dbscan_clusterer.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>

namespace lidar_perception {

DBSCANClusterer::DBSCANClusterer(float eps, int min_pts, int min_cluster_size, int max_cluster_size)
    : eps_(eps), min_pts_(min_pts), min_cluster_size_(min_cluster_size), max_cluster_size_(max_cluster_size) {}

void DBSCANClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    const int n_points = static_cast<int>(cloud->size());
    const float inv_eps = 1.0f / eps_;
    
    const float range_xy = 30.0f;
    const float range_z = 2.0f;
    const int dim_xy = static_cast<int>(2.0f * range_xy * inv_eps) + 1;
    const int dim_z = static_cast<int>(2.0f * range_z * inv_eps) + 1;
    const int grid_size = dim_xy * dim_xy * dim_z;

    // Initialize or resize persistent grid
    if (grid_.size() != static_cast<size_t>(grid_size)) {
        grid_.assign(grid_size, std::vector<int>());
    } else {
        for (auto& cell : grid_) cell.clear();
    }

    auto get_idx = [&](float x, float y, float z) {
        int gx = static_cast<int>((x + range_xy) * inv_eps);
        int gy = static_cast<int>((y + range_xy) * inv_eps);
        int gz = static_cast<int>((z + range_z) * inv_eps);
        if (gx < 0 || gx >= dim_xy || gy < 0 || gy >= dim_xy || gz < 0 || gz >= dim_z) return -1;
        return (gx * dim_xy * dim_z) + (gy * dim_z) + gz;
    };

    for (int i = 0; i < n_points; ++i) {
        int idx = get_idx(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        if (idx != -1) grid_[idx].push_back(i);
    }

    labels_.assign(n_points, -1); 
    int current_cluster_id = 0;
    std::queue<int> seed_queue;
    const float eps_sq = eps_ * eps_;

    auto get_neighbors = [&](int idx, std::vector<int>& neighbors) {
        neighbors.clear();
        const auto& pt = cloud->points[idx];
        int gx = static_cast<int>((pt.x + range_xy) * inv_eps);
        int gy = static_cast<int>((pt.y + range_xy) * inv_eps);
        int gz = static_cast<int>((pt.z + range_z) * inv_eps);

        for (int x = gx - 1; x <= gx + 1; ++x) {
            if (x < 0 || x >= dim_xy) continue;
            for (int y = gy - 1; y <= gy + 1; ++y) {
                if (y < 0 || y >= dim_xy) continue;
                for (int z = gz - 1; z <= gz + 1; ++z) {
                    if (z < 0 || z >= dim_z) continue;
                    
                    int g_idx = (x * dim_xy * dim_z) + (y * dim_z) + z;
                    for (int n_idx : grid_[g_idx]) {
                        const auto& npt = cloud->points[n_idx];
                        float dx = pt.x - npt.x;
                        float dy = pt.y - npt.y;
                        float dz = pt.z - npt.z;
                        if (dx*dx + dy*dy + dz*dz <= eps_sq) {
                            neighbors.push_back(n_idx);
                        }
                    }
                }
            }
        }
    };

    neighbor_indices_.reserve(200);

    for (int i = 0; i < n_points; ++i) {
        if (labels_[i] != -1) continue;

        get_neighbors(i, neighbor_indices_);

        if (static_cast<int>(neighbor_indices_.size()) < min_pts_) {
            labels_[i] = -2; 
            continue;
        }

        labels_[i] = current_cluster_id;
        for (int n_idx : neighbor_indices_) {
            if (labels_[n_idx] < 0) {
                labels_[n_idx] = current_cluster_id;
                seed_queue.push(n_idx);
            }
        }

        while (!seed_queue.empty()) {
            int curr = seed_queue.front();
            seed_queue.pop();

            std::vector<int> curr_neighbors;
            get_neighbors(curr, curr_neighbors);

            if (static_cast<int>(curr_neighbors.size()) >= min_pts_) {
                for (int n_idx : curr_neighbors) {
                    if (labels_[n_idx] < 0) {
                        labels_[n_idx] = current_cluster_id;
                        seed_queue.push(n_idx);
                    }
                }
            }
        }
        current_cluster_id++;
    }

    std::vector<PointCloudPtr> temp_clusters(current_cluster_id);
    for (int i = 0; i < current_cluster_id; ++i) temp_clusters[i].reset(new PointCloud);
    for (int i = 0; i < n_points; ++i) {
        if (labels_[i] >= 0) temp_clusters[labels_[i]]->push_back(cloud->points[i]);
    }

    for (auto& c : temp_clusters) {
        if (c->size() >= static_cast<size_t>(min_cluster_size_) && c->size() <= static_cast<size_t>(max_cluster_size_)) {
            clusters.push_back(c);
        }
    }
}

} // namespace lidar_perception
