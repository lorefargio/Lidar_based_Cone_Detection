/**
 * @file bin_based_ground_remover.hpp
 * @brief High-efficiency terrain segmentation using polar binning and height thresholds.
 * 
 * This module implements a computationally efficient ground removal strategy by 
 * partitioning the coordinate space into a polar grid. It identifies local 
 * elevation minima within each bin to distinguish the traversable surface 
 * from non-ground obstacles.
 */

#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <vector>
#include <limits>

namespace fs_perception {

/**
 * @class BinBasedGroundRemover
 * @brief Fast terrain segmentation using polar grid discretization.
 * 
 * The BinBasedGroundRemover partitions the point cloud into radial angular segments 
 * and distance bins. For each discrete bin, a local height minimum is identified. 
 * Points exhibiting a vertical displacement from this local minimum below a 
 * predefined threshold are classified as ground.
 * 
 * @note This algorithm is highly performant (O(N)) and suitable for real-time 
 * execution on mobile hardware.
 */
class BinBasedGroundRemover : public GroundRemoverInterface {
public:
    /**
     * @struct Config
     * @brief Parameters governing the polar binning and height segmentation logic.
     */
    struct Config {
        float max_range = 25.0f;           ///< The maximum radial distance for point inclusion (meters).
        float sensor_z = -0.50f;           ///< The vertical offset of the LiDAR origin relative to the ground (meters).
        float hard_ground_cutoff = -0.47f; ///< Absolute vertical threshold below which points are unconditionally ground.
        float local_threshold = 0.02f;     ///< Permissible vertical deviation from the bin's local minimum height (meters).
        int segments = 500;                ///< Total number of radial angular divisions.
        int bins = 500;                    ///< Total number of radial distance divisions.
    };

    /**
     * @brief Constructs a BinBasedGroundRemover with default parameters.
     */
    BinBasedGroundRemover();

    /**
     * @brief Constructs a BinBasedGroundRemover with specialized configuration.
     * 
     * @param config The configuration structure containing binning and threshold parameters.
     */
    explicit BinBasedGroundRemover(const Config& config);

    /**
     * @brief Segregates the input point cloud into obstacle and ground subsets.
     * 
     * Implements the core polar binning logic to categorize points based on local 
     * height minima and global vertical constraints.
     * 
     * @param[in]  cloud_in The pre-processed input LiDAR point cloud.
     * @param[out] cloud_obstacles The resulting subset of non-ground obstacle points.
     * @param[out] cloud_ground The resulting subset of terrain points.
     */
    void removeGround(const PointCloudConstPtr& cloud_in, 
                      PointCloudPtr& cloud_obstacles, 
                      PointCloudPtr& cloud_ground) override;

private:
    /**
     * @struct Bin
     * @brief Internal representation of a single polar grid cell's state.
     */
    struct Bin {
        float min_z = std::numeric_limits<float>::max(); ///< The lowest vertical coordinate recorded in this bin.
        bool has_points = false;                         ///< Indicates if any point has been mapped to this cell.
    };

    Config config_;             ///< Operational configuration parameters.
    std::vector<Bin> grid_;     ///< A flattened vector representing the 2D polar bin structure.
};

} // namespace fs_perception