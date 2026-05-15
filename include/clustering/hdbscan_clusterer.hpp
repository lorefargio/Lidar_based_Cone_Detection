#pragma once

#include "clustering/clusterer_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace lidar_perception {

/**
 * @class HDBSCANClusterer
 * @brief Lidar-Aware Hierarchical Density-Based Spatial Clustering.
 * 
 * Optimized: builds a Minimum Spanning Forest to capture 
 * all isolated cones and uses a distance-scaled Mutual Reachability metric.
 */
class HDBSCANClusterer : public ClustererInterface {
public:
    struct Config {
        int min_pts = 3;              ///< Neighbors for density calculation (matching DBSCAN).
        int min_cluster_size = 3;     ///< Min points for a valid cone.
        int max_cluster_size = 300;   ///< Max points for a valid cone.
        float alpha_scaling = 0.01f;  ///< Subtle beam divergence compensation.
    };

    HDBSCANClusterer();
    explicit HDBSCANClusterer(const Config& config);

    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    Config config_;
    pcl::search::KdTree<PointT>::Ptr tree_;

    struct Edge {
        int u, v;
        float weight;
        bool operator<(const Edge& other) const { return weight < other.weight; }
    };

    void extract(const PointCloudPtr& cloud, const std::vector<int>& indices, std::vector<PointCloudPtr>& clusters);

    // Persistent buffers
    std::vector<float> core_distances_;
    std::vector<int> nn_indices_;
    std::vector<float> nn_dists_;
    std::vector<Edge> edges_;
    std::vector<int> labels_;
    std::vector<bool> extracted_;

    struct UnionFind {
        std::vector<int> parent;
        std::vector<int> size;
        UnionFind(int n) {
            parent.resize(n);
            size.assign(n, 1);
            for(int i=0; i<n; ++i) parent[i] = i;
        }
        int find(int i) {
            if (parent[i] == i) return i;
            return parent[i] = find(parent[i]);
        }
        // Unite returns true if merged, false if already same or should not merge
        bool unite(int i, int j, int max_size) {
            int root_i = find(i);
            int root_j = find(j);
            if (root_i != root_j) {
                if (size[root_i] + size[root_j] <= max_size) {
                    parent[root_i] = root_j;
                    size[root_j] += size[root_i];
                    return true;
                }
            }
            return false;
        }
    };
};

} // namespace lidar_perception
