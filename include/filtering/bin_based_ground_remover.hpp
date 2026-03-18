#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <vector>
#include <limits>

namespace fs_perception {

/**
 * @class BinBasedGroundRemover
 * @brief Fast ground removal algorithm based on a polar grid and local height threshold.
 * 
 * Divides the cloud into a grid of radial sectors and bins. For each bin, it finds 
 * the lowest point and classifies any point within a certain height above it as ground.
 */
class BinBasedGroundRemover : public GroundRemoverInterface {
public:
    /**
     * @struct Config
     * @brief Parameters for the bin-based ground removal algorithm.
     */
    struct Config {
        float max_range = 25.0f;           ///< Maximum radial distance for processing.
        float sensor_z = -0.42f;           ///< Height of the sensor from the ground (meters).
        float hard_ground_cutoff = -0.47f; ///< Absolute height cutoff (all points below are ground).
        float local_threshold = 0.02f;     ///< Maximum height difference from the local minimum to consider a point ground.
        int segments = 500;                ///< Number of radial angular segments.
        int bins = 500;                    ///< Number of radial distance bins.
    };

    /**
     * @brief Default constructor for BinBasedGroundRemover.
     */
    BinBasedGroundRemover();

    /**
     * @brief Explicit constructor with custom configuration.
     * @param config The custom configuration struct.
     */
    explicit BinBasedGroundRemover(const Config& config);

    /**
     * @brief Separates ground points from obstacle points.
     * @param cloud_in[in] Original point cloud.
     * @param cloud_obstacles[out] Output cloud containing obstacle candidates.
     * @param cloud_ground[out] Output cloud containing ground points.
     */
    void removeGround(const PointCloudConstPtr& cloud_in, 
                      PointCloudPtr& cloud_obstacles, 
                      PointCloudPtr& cloud_ground) override;

private:
    /**
     * @struct Bin
     * @brief Local storage for each grid bin's state.
     */
    struct Bin {
        float min_z = std::numeric_limits<float>::max(); ///< Minimum height found in this bin.
        bool has_points = false;                         ///< Whether any points were mapped to this bin.
    };

    Config config_;             ///< Algorithm configuration.
    std::vector<Bin> grid_;     ///< Memory structure representing the polar grid.
};

} // namespace fs_perception