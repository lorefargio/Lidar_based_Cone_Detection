#include "clustering/adaptive_dbscan_clusterer.hpp"
#include <queue>
#include <cmath>

namespace fs_perception {

AdaptiveDBSCANClusterer::AdaptiveDBSCANClusterer(float eps_base, float alpha, int min_pts, int min_cluster_size, int max_cluster_size)
    : eps_base_(eps_base), alpha_(alpha), min_pts_(min_pts), min_cluster_size_(min_cluster_size), max_cluster_size_(max_cluster_size) {}

void AdaptiveDBSCANClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) {
        return;
    }

    // Initialize the search tree for efficient neighbor lookup
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<bool> visited(cloud->size(), false);

    // Dynamic epsilon adjustment based on radial distance
    auto get_dynamic_eps = [&](const PointT& pt) {
        float distance = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        return eps_base_ + alpha_ * distance;
    };

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (visited[i]) continue;
        visited[i] = true;

        float current_eps = get_dynamic_eps(cloud->points[i]);
        std::vector<int> nn_indices;
        std::vector<float> nn_dists;

        // Core point check
        if (tree->radiusSearch(cloud->points[i], current_eps, nn_indices, nn_dists) >= min_pts_) {
            PointCloudPtr current_cluster(new PointCloud);
            current_cluster->push_back(cloud->points[i]);

            // BFS expansion from the core point
            std::queue<int> seed_queue;
            for (size_t j = 1; j < nn_indices.size(); ++j) {
                int idx = nn_indices[j];
                if (!visited[idx]) {
                    visited[idx] = true;
                    seed_queue.push(idx);
                }
            }

            while (!seed_queue.empty()) {
                int curr_idx = seed_queue.front();
                seed_queue.pop();
                current_cluster->push_back(cloud->points[curr_idx]);

                float local_eps = get_dynamic_eps(cloud->points[curr_idx]);
                std::vector<int> curr_nn_indices;
                std::vector<float> curr_nn_dists;
                
                // Expand if the neighbor is also a core point
                if (tree->radiusSearch(cloud->points[curr_idx], local_eps, curr_nn_indices, curr_nn_dists) >= min_pts_) {
                    for (size_t j = 1; j < curr_nn_indices.size(); ++j) {
                        int idx = curr_nn_indices[j];
                        if (!visited[idx]) {
                            visited[idx] = true;
                            seed_queue.push(idx);
                        }
                    }
                }
            }

            // Apply size filters before saving the cluster
            if (current_cluster->size() >= (size_t)min_cluster_size_ && current_cluster->size() <= (size_t)max_cluster_size_) {
                clusters.push_back(current_cluster);
            }
        }
    }
}

} // namespace fs_perception
