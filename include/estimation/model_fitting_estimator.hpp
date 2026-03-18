#pragma once

#include "estimation/estimator_interface.hpp"

namespace fs_perception {

/**
 * @class ModelFittingEstimator
 * @brief Estimator that uses geometric model fitting (e.g., RANSAC) to classify cones.
 * 
 * Uses the Random Sample Consensus algorithm to fit a cylinder model onto a 
 * point cluster, making it robust against noise and residual ground points.
 */
class ModelFittingEstimator : public EstimatorInterface {
public:
    /**
     * @struct Config
     * @brief Configuration parameters for the model fitting algorithm.
     */
    struct Config {
        float normal_distance_weight = 0.1f; ///< Weight assigned to surface normals for cylinder fitting.
        int max_iterations = 1000;          ///< Maximum RANSAC iterations.
        float distance_threshold = 0.05f;   ///< Threshold for considering a point an inlier.
        float radius_min = 0.05f;           ///< Minimum allowed cylinder radius (meters).
        float radius_max = 0.18f;           ///< Maximum allowed cylinder radius (meters).
        float min_inlier_ratio = 0.4f;      ///< Minimum ratio of inliers to total cluster size.
        float min_height = 0.15f;           ///< Minimum allowed cluster height (meters).
        float max_height = 0.35f;           ///< Maximum allowed cluster height (meters).
        
        // PCA Pre-filtering thresholds
        float max_linearity = 0.9f;         ///< Maximum allowed linearity (to reject posts/legs).
        float min_scatter = 0.02f;          ///< Minimum allowed scattering (to ensure volumetric shape).
    };

    /**
     * @brief Default constructor for ModelFittingEstimator.
     */
    ModelFittingEstimator();

    /**
     * @brief Explicit constructor with custom configuration.
     * @param config The custom configuration struct.
     */
    explicit ModelFittingEstimator(const Config& config);

    /**
     * @brief Estimates whether a point cluster represents a cone by fitting a cylinder model.
     * @param cluster Point cloud of a single object candidate.
     * @return Cone structure with estimated pose and confidence based on inlier ratio.
     */
    Cone estimate(const PointCloudPtr& cluster) override;

private:
    Config config_; ///< Current configuration parameters.
};

} // namespace fs_perception