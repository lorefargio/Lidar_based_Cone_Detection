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

    struct UnionFind {
        std::vector<int> parent;
        UnionFind(int n) {
            parent.resize(n);
            for(int i=0; i<n; ++i) parent[i] = i;
        }
        int find(int i) {
            if (parent[i] == i) return i;
            return parent[i] = find(parent[i]);
        }
        void unite(int i, int j) {
            int root_i = find(i);
            int root_j = find(j);
            if (root_i != root_j) parent[root_i] = root_j;
        }
    };
};

} // namespace lidar_perception
