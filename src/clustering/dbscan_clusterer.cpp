#include "clustering/dbscan_clusterer.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>

namespace fs_perception {

DBSCANClusterer::DBSCANClusterer(float eps, int min_pts, int min_cluster_size, int max_cluster_size)
    : eps_(eps), min_pts_(min_pts), min_cluster_size_(min_cluster_size), max_cluster_size_(max_cluster_size) {}

void DBSCANClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    const int n_points = static_cast<int>(cloud->size());
    const float inv_eps = 1.0f / eps_;
    
    // Spatial Hash Grid for O(1) neighbor lookup
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
            static_cast<int>(std::floor(pt.x * inv_eps)),
            static_cast<int>(static_cast<int>(std::floor(pt.y * inv_eps))),
            static_cast<int>(static_cast<int>(std::floor(pt.z * inv_eps)))
        };
        grid[key].push_back(i);
    }

    std::vector<int> labels(n_points, -1); // -1: unvisited, -2: noise, >=0: cluster_id
    int current_cluster_id = 0;
    
    std::vector<int> neighbor_indices;
    neighbor_indices.reserve(100);
    std::queue<int> seed_queue;

    const float eps_sq = eps_ * eps_;

    auto get_neighbors = [&](int idx, std::vector<int>& neighbors) {
        neighbors.clear();
        const auto& pt = cloud->points[idx];
        int gx = static_cast<int>(std::floor(pt.x * inv_eps));
        int gy = static_cast<int>(std::floor(pt.y * inv_eps));
        int gz = static_cast<int>(std::floor(pt.z * inv_eps));

        for (int x = gx - 1; x <= gx + 1; ++x) {
            for (int y = gy - 1; y <= gy + 1; ++y) {
                for (int z = gz - 1; z <= gz + 1; ++z) {
                    auto it = grid.find({x, y, z});
                    if (it != grid.end()) {
                        for (int neighbor_idx : it->second) {
                            const auto& npt = cloud->points[neighbor_idx];
                            float dx = pt.x - npt.x;
                            float dy = pt.y - npt.y;
                            float dz = pt.z - npt.z;
                            if (dx*dx + dy*dy + dz*dz <= eps_sq) {
                                neighbors.push_back(neighbor_idx);
                            }
                        }
                    }
                }
            }
        }
    };

    for (int i = 0; i < n_points; ++i) {
        if (labels[i] != -1) continue;

        get_neighbors(i, neighbor_indices);

        if (static_cast<int>(neighbor_indices.size()) < min_pts_) {
            labels[i] = -2; // Noise
            continue;
        }

        // Start new cluster
        labels[i] = current_cluster_id;
        for (int neighbor_idx : neighbor_indices) {
            if (labels[neighbor_idx] == -1 || labels[neighbor_idx] == -2) {
                labels[neighbor_idx] = current_cluster_id;
                seed_queue.push(neighbor_idx);
            }
        }

        while (!seed_queue.empty()) {
            int curr_idx = seed_queue.front();
            seed_queue.pop();

            std::vector<int> curr_neighbors;
            get_neighbors(curr_idx, curr_neighbors);

            if (static_cast<int>(curr_neighbors.size()) >= min_pts_) {
                for (int neighbor_idx : curr_neighbors) {
                    if (labels[neighbor_idx] == -1 || labels[neighbor_idx] == -2) {
                        labels[neighbor_idx] = current_cluster_id;
                        seed_queue.push(neighbor_idx);
                    }
                }
            }
        }
        current_cluster_id++;
    }

    // Convert labels to point clouds
    std::vector<PointCloudPtr> temp_clusters(current_cluster_id);
    for (int i = 0; i < current_cluster_id; ++i) {
        temp_clusters[i].reset(new PointCloud);
    }

    for (int i = 0; i < n_points; ++i) {
        if (labels[i] >= 0) {
            temp_clusters[labels[i]]->push_back(cloud->points[i]);
        }
    }

    for (auto& c : temp_clusters) {
        if (c->size() >= static_cast<size_t>(min_cluster_size_) && c->size() <= static_cast<size_t>(max_cluster_size_)) {
            clusters.push_back(c);
        }
    }
}

} // namespace fs_perception
