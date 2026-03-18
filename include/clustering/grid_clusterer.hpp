#pragma once

#include "clustering/clusterer_interface.hpp"
#include <vector>
#include <unordered_map>

namespace fs_perception {

class GridClusterer : public ClustererInterface {
public:
    /**
     * @brief Grid-based 2D Connected Components clustering. 
     * Extremely fast (O(N)) and robust for Lidar obstacle detection.
     * @param grid_resolution Size of each grid cell (e.g., 0.15m = 15cm)
     * @param max_range Max distance to process (e.g., 25.0m)
     */
    GridClusterer(float grid_resolution = 0.15f, float max_range = 25.0f, int min_cluster_size = 3, int max_cluster_size = 300);

    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float grid_res_;
    float max_range_;
    int min_size_;
    int max_size_;

    struct Cell {
        int id = -1; // -1 means unvisited, >0 means cluster ID
        std::vector<int> point_indices;
    };
    
    // Hash function for 2D grid coordinates
    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator () (const std::pair<T1,T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ (h2 << 1); 
        }
    };
};

} // namespace fs_perception