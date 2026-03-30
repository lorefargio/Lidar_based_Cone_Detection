#pragma once

#include "clustering/clusterer_interface.hpp"
#include <vector>
#include <unordered_map>

namespace fs_perception {

/**
 * @class GridClusterer
 * @brief Fast 2D Grid-based Connected Components clustering. 
 * 
 * Projects points onto a sparse 2D grid and finds connected components (8-connectivity) 
 * among occupied cells. This algorithm is extremely fast (O(N)).
 */
class GridClusterer : public ClustererInterface {
public:
    /**
     * @brief Constructor for GridClusterer.
     * @param grid_resolution Size of each grid cell (meters).
     * @param max_range Max distance to process (meters).
     * @param min_cluster_size Minimum points for a valid cluster.
     * @param max_cluster_size Maximum points for a valid cluster.
     */
    GridClusterer(float grid_resolution = 0.15f, float max_range = 25.0f, int min_cluster_size = 3, int max_cluster_size = 300);

    /**
     * @brief Performs 2D grid clustering on the provided point cloud.
     * @param cloud Input obstacle point cloud.
     * @param clusters Identified clusters.
     */
    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float grid_res_;          ///< Resolution of the grid cells.
    float max_range_;         ///< Maximum distance for points from origin.
    int min_size_;            ///< Minimum number of points in a cluster.
    int max_size_;            ///< Maximum number of points in a cluster.

    /**
     * @struct Cell
     * @brief Represents a single cell in the sparse 2D grid.
     */
    struct Cell {
        int id = -1;                      ///< Cluster ID (-1 if unvisited).
        std::vector<int> point_indices;   ///< Indices of points falling in this cell.
    };

    std::vector<Cell> grid_;  ///< Pre-allocated flat grid.
    int grid_dim_;            ///< Number of cells in one dimension.
    
    inline int getGridIndex(int x, int y) const {
        return (x + grid_dim_/2) * grid_dim_ + (y + grid_dim_/2);
    }
};

} // namespace fs_perception
