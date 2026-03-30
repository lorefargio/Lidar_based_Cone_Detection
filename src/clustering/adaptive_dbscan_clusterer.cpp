#include "clustering/adaptive_dbscan_clusterer.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace fs_perception {

AdaptiveDBSCANClusterer::AdaptiveDBSCANClusterer(float eps_base, float alpha, int min_pts, int min_cluster_size, int max_cluster_size)
    : eps_base_(eps_base), alpha_(alpha), min_pts_(min_pts), min_cluster_size_(min_cluster_size), max_cluster_size_(max_cluster_size) {}

void AdaptiveDBSCANClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    const int n_points = static_cast<int>(cloud->size());
    
    // 1. Spatial Hash Grid for FAST proximity lookup
    // Since epsilon is dynamic, we use a grid size equal to the maximum possible epsilon
    // for the expected max_range (e.g., at 25m).
    const float max_expected_eps = eps_base_ + alpha_ * 25.0f;
    const float inv_grid_size = 1.0f / max_expected_eps;

    struct GridKey {
        int x, y, z;
        bool operator==(const GridKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    struct GridKeyHasher {
        std::size_t operator()(const GridKey& k) const {
            return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
        }
    };

    std::unordered_map<GridKey, std::vector<int>, GridKeyHasher> grid;
    grid.reserve(n_points);

    for (int i = 0; i < n_points; ++i) {
        const auto& pt = cloud->points[i];
        GridKey key{
            static_cast<int>(std::floor(pt.x * inv_grid_size)),
            static_cast<int>(std::floor(pt.y * inv_grid_size)),
            static_cast<int>(std::floor(pt.z * inv_grid_size))
        };
        grid[key].push_back(i);
    }

    std::vector<int> labels(n_points, -1);
    int current_cluster_id = 0;
    std::queue<int> seed_queue;

    // Helper to calculate distance-aware min_pts and epsilon
    auto get_params = [&](const PointT& pt) {
        float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        float eps = eps_base_ + alpha_ * r;
        
        // Far away cones have fewer points, scale min_pts down to 3 at 15m+
        int min_p = std::max(3, static_cast<int>(min_pts_ * std::exp(-0.05f * r)));
        
        return std::make_pair(eps, min_p);
    };

    auto get_neighbors = [&](int idx, std::vector<int>& neighbors) {
        neighbors.clear();
        const auto& pt = cloud->points[idx];
        auto [eps, min_p] = get_params(pt);
        const float eps_sq = eps * eps;

        int gx = static_cast<int>(std::floor(pt.x * inv_grid_size));
        int gy = static_cast<int>(std::floor(pt.y * inv_grid_size));
        int gz = static_cast<int>(std::floor(pt.z * inv_grid_size));

        // We only need to check the current cell and immediate neighbors because 
        // max_expected_eps >= current_eps.
        for (int x = gx - 1; x <= gx + 1; ++x) {
            for (int y = gy - 1; y <= gy + 1; ++y) {
                for (int z = gz - 1; z <= gz + 1; ++z) {
                    auto it = grid.find({x, y, z});
                    if (it != grid.end()) {
                        for (int n_idx : it->second) {
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
        }
        return min_p;
    };

    std::vector<int> neighbor_indices;
    neighbor_indices.reserve(100);

    for (int i = 0; i < n_points; ++i) {
        if (labels[i] != -1) continue;

        int min_p = get_neighbors(i, neighbor_indices);

        if (static_cast<int>(neighbor_indices.size()) < min_p) {
            labels[i] = -2;
            continue;
        }

        labels[i] = current_cluster_id;
        for (int n_idx : neighbor_indices) {
            if (labels[n_idx] < 0) {
                labels[n_idx] = current_cluster_id;
                seed_queue.push(n_idx);
            }
        }

        while (!seed_queue.empty()) {
            int curr = seed_queue.front();
            seed_queue.pop();

            std::vector<int> curr_neighbors;
            int curr_min_p = get_neighbors(curr, curr_neighbors);

            if (static_cast<int>(curr_neighbors.size()) >= curr_min_p) {
                for (int n_idx : curr_neighbors) {
                    if (labels[n_idx] < 0) {
                        labels[n_idx] = current_cluster_id;
                        seed_queue.push(n_idx);
                    }
                }
            }
        }
        current_cluster_id++;
    }

    // Extraction
    std::vector<PointCloudPtr> temp_clusters(current_cluster_id);
    for (int i = 0; i < current_cluster_id; ++i) temp_clusters[i].reset(new PointCloud);
    for (int i = 0; i < n_points; ++i) if (labels[i] >= 0) temp_clusters[labels[i]]->push_back(cloud->points[i]);

    for (auto& c : temp_clusters) {
        if (c->size() >= static_cast<size_t>(min_cluster_size_) && c->size() <= static_cast<size_t>(max_cluster_size_)) {
            clusters.push_back(c);
        }
    }
}

} // namespace fs_perception
