#include "clustering/hdbscan_clusterer.hpp"
#include <pcl/common/common.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_perception {

HDBSCANClusterer::HDBSCANClusterer() : HDBSCANClusterer(Config()) {}

HDBSCANClusterer::HDBSCANClusterer(const Config& config) 
    : config_(config), tree_(new pcl::search::KdTree<PointT>) {}

void HDBSCANClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    const int n = static_cast<int>(cloud->size());
    tree_->setInputCloud(cloud);

    // 1. Core Distance Calculation
    core_distances_.assign(n, 0.0f);
    nn_indices_.resize(config_.min_pts);
    nn_dists_.resize(config_.min_pts);

    for (int i = 0; i < n; ++i) {
        const auto& pt = cloud->points[i];
        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        
        if (tree_->nearestKSearch(i, config_.min_pts, nn_indices_, nn_dists_) >= config_.min_pts) {
            float raw_core_dist = std::sqrt(nn_dists_.back());
            core_distances_[i] = raw_core_dist / (1.0f + config_.alpha_scaling * r);
        } else {
            core_distances_[i] = 2.0f; 
        }
    }

    // 2. Build MST Edges
    edges_.clear();
    const int M = 8; // Sufficient for cone structure
    edges_.reserve(n * M);

    for (int i = 0; i < n; ++i) {
        if (tree_->nearestKSearch(i, M, nn_indices_, nn_dists_) > 0) {
            float r_u = std::sqrt(cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y);
            float scale = 1.0f + config_.alpha_scaling * r_u;
            for (size_t j = 1; j < nn_indices_.size(); ++j) {
                int v = nn_indices_[j];
                float mreach_dist = std::max({core_distances_[i], core_distances_[v], std::sqrt(nn_dists_[j]) / scale});
                if (i < v) edges_.push_back({i, v, mreach_dist});
            }
        }
    }

    // 3. Kruskal's with Size Constraint
    std::sort(edges_.begin(), edges_.end());
    
    UnionFind uf(n);
    for (const auto& edge : edges_) {
        uf.unite(edge.u, edge.v, config_.max_cluster_size);
    }

    // 4. Cluster Extraction
    std::unordered_map<int, std::vector<int>> root_to_indices;
    for (int i = 0; i < n; ++i) {
        int root = uf.find(i);
        if (uf.size[root] >= config_.min_cluster_size) {
            root_to_indices[root].push_back(i);
        }
    }

    for (auto& pair : root_to_indices) {
        extract(cloud, pair.second, clusters);
    }
}

void HDBSCANClusterer::extract(const PointCloudPtr& cloud, const std::vector<int>& indices, std::vector<PointCloudPtr>& clusters) {
    PointCloudPtr cluster(new PointCloud);
    cluster->points.reserve(indices.size());
    for (int idx : indices) cluster->push_back(cloud->points[idx]);
    cluster->width = static_cast<uint32_t>(cluster->size());
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
}

} // namespace lidar_perception
