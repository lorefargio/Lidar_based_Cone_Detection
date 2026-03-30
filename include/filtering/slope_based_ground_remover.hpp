#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <vector>
#include <cmath>

namespace fs_perception {

/**
 * @class SlopeBasedGroundRemover
 * @brief Ground removal algorithm based on radial slope analysis.
 * 
 * This algorithm operates in two main stages for efficiency and robustness:
 * 1. **Angular Binning**: The point cloud is partitioned into vertical wedges (sectors)
 *    to reduce the complexity of the global problem into independent 1D problems.
 * 2. **Radial Slope Logic**: Points in each sector are sorted by distance from the 
 *    sensor. The slope (dz / dr) between consecutive points is compared against 
 *    thresholds. This allows the algorithm to follow terrain gradients (e.g., track 
 *    inclines) instead of using a global height cutoff.
 */
class SlopeBasedGroundRemover : public GroundRemoverInterface {
public:
    /**
     * @struct Config
     * @brief Parameters governing the slope-based classification logic.
     */
    struct Config {
        float max_range = 25.0f;           ///< Maximum radial distance for processing.
        float sensor_z = -0.52f;           ///< Height of the sensor from the ground (meters).
        float max_slope = 0.08f;           ///< Maximum slope (dz / dr) tolerated for the ground.
        float max_z_diff = 0.05f;          ///< Maximum height difference between consecutive ground points.
        float initial_ground_threshold = 0.05f; ///< Initial ground threshold near the sensor.
        int segments = 360;                ///< Angular resolution (number of radial sectors).
    };

    /**
     * @brief Default constructor for SlopeBasedGroundRemover.
     */
    SlopeBasedGroundRemover();

    /**
     * @brief Explicit constructor with custom configuration.
     * @param config The custom configuration struct.
     */
    explicit SlopeBasedGroundRemover(const Config& config);

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
    Config config_; ///< Algorithm configuration.
    std::vector<std::vector<int>> sectors_; ///< Persistent sector buffer.
};

} // namespace fs_perception