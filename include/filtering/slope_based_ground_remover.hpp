#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <vector>
#include <cmath>

namespace fs_perception {

/**
 * @class SlopeBasedGroundRemover
 * @brief Ground removal algorithm based on radial slope analysis.
 * 
 * Analyzes points in radial sectors and sorts them by distance. It calculates 
 * the slope between consecutive points and classifies them as ground if 
 * the slope is within a certain threshold, effectively handling terrain slopes.
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
};

} // namespace fs_perception