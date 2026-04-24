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

    // Phase 1: Range-Normalized Core Distance
    // Calcoliamo la densità locale compensando la divergenza del fascio del Pandar40P.
    std::vector<float> core_distances(n);
    std::vector<int> nn_indices(config_.min_pts);
    std::vector<float> nn_dists(config_.min_pts);

    for (int i = 0; i < n; ++i) {
        const auto& pt = cloud->points[i];
        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        
        if (tree_->nearestKSearch(i, config_.min_pts, nn_indices, nn_dists) >= config_.min_pts) {
            float raw_core_dist = std::sqrt(nn_dists.back());
            // Normalizzazione: rendiamo la densità indipendente dal range.
            // La divergenza del fascio aumenta linearmente con r.
            core_distances[i] = raw_core_dist / (1.0f + config_.alpha_scaling * r);
        } else {
            core_distances[i] = 2.0f; 
        }
    }

    // Phase 2: Build K-NN Graph with Normalized Mutual Reachability
    std::vector<Edge> edges;
    const int M = 10; // Vicini sufficienti per catturare la struttura di un cono
    edges.reserve(n * M);

    for (int i = 0; i < n; ++i) {
        if (tree_->nearestKSearch(i, M, nn_indices, nn_dists) > 0) {
            for (size_t j = 1; j < nn_indices.size(); ++j) {
                int v = nn_indices[j];
                float dist_uv = std::sqrt(nn_dists[j]);
                float r_u = std::sqrt(cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y);
                
                // Normalizziamo anche la distanza tra i punti
                float dist_norm = dist_uv / (1.0f + config_.alpha_scaling * r_u);
                
                // Mutual Reachability Distance (Normalized)
                float mreach_dist = std::max({core_distances[i], core_distances[v], dist_norm});

                if (i < v) edges.push_back({i, v, mreach_dist});
            }
        }
    }

    // Phase 3: Single Linkage & Forest Extraction
    std::sort(edges.begin(), edges.end());
    
    UnionFind uf(n);
    std::vector<std::vector<int>> components(n);
    for(int i=0; i<n; ++i) components[i].push_back(i);
    
    std::vector<bool> extracted(n, false);

    for (const auto& edge : edges) {
        int root_u = uf.find(edge.u);
        int root_v = uf.find(edge.v);
        
        if (root_u != root_v) {
            size_t size_u = components[root_u].size();
            size_t size_v = components[root_v].size();
            
            // Se l'unione eccede la dimensione di un cono, salviamo i pezzi validi
            if (size_u + size_v > static_cast<size_t>(config_.max_cluster_size)) {
                if (size_u >= static_cast<size_t>(config_.min_cluster_size) && !extracted[root_u]) {
                    extract(cloud, components[root_u], clusters);
                    extracted[root_u] = true;
                }
                if (size_v >= static_cast<size_t>(config_.min_cluster_size) && !extracted[root_v]) {
                    extract(cloud, components[root_v], clusters);
                    extracted[root_v] = true;
                }
                continue;
            }

            // Merge dei componenti
            uf.unite(root_u, root_v);
            int new_root = uf.find(root_u);
            int other_root = (new_root == root_u) ? root_v : root_u;
            
            components[new_root].insert(components[new_root].end(), components[other_root].begin(), components[other_root].end());
            components[other_root].clear();
            extracted[new_root] = extracted[new_root] || extracted[other_root];
        }
    }

    // Check finale per componenti isolati rimasti
    for (int i = 0; i < n; ++i) {
        if (!components[i].empty() && !extracted[i] &&
            components[i].size() >= static_cast<size_t>(config_.min_cluster_size)) {
            extract(cloud, components[i], clusters);
        }
    }
}

void HDBSCANClusterer::extract(const PointCloudPtr& cloud, const std::vector<int>& indices, std::vector<PointCloudPtr>& clusters) {
    PointCloudPtr cluster(new PointCloud);
    cluster->points.reserve(indices.size());
    for (int idx : indices) cluster->push_back(cloud->points[idx]);
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
}

} // namespace lidar_perception
