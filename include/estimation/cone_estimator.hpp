#pragma once

#include "utils/types.hpp"

namespace fs_perception {

/**
 * @class ConeEstimator
 * @brief Classifies cones based on a set of geometric and statistical rules.
 * 
 * This estimator evaluates candidates using bounding box size, aspect ratio, 
 * PCA-based shape features (Linearity, Planarity, Scattering), and intensity 
 * with dynamic thresholds that adapt to the object's distance from the sensor.
 */
class ConeEstimator {
public:
    /**
     * @struct Config
     * @brief Parameters governing the rule-based classification logic.
     */
    struct Config {
        float ground_z_level = -0.52f;      ///< Estimated ground level (m) to calculate object height.
        float dynamic_width_decay = 0.005f; ///< Decrement for minimum width threshold per meter.
        int min_points_at_10m = 10;         ///< Minimum points expected for a cone at 10 meters distance.
        
        float min_height = 0.10f;           ///< Minimum height threshold (meters).
        float max_height = 0.50f;           ///< Maximum height threshold (meters).
        float base_min_width = 0.10f;       ///< Base minimum width threshold (meters) at distance 0.
        float max_width = 0.36f;            ///< Maximum allowed width (meters).
        float min_aspect_ratio = 0.4f;      ///< Minimum width-to-height ratio.
        float max_aspect_ratio = 2.0f;      ///< Maximum width-to-height ratio.
        float max_width_diff_ratio = 3.5f;  ///< Maximum allowed ratio between the largest and smallest horizontal widths.
        float min_intensity = 5.0f;         ///< Minimum average intensity threshold.
        float yellow_intensity_threshold = 25.0f; ///< Intensity above which a cone is classified as YELLOW.
        float color_classification_range = 5.0f;  ///< Maximum range to attempt color classification (meters).

        // Principal Component Analysis (PCA) thresholds for shape classification
        float max_linearity = 0.8f;         ///< Threshold to reject highly linear objects (e.g., posts, legs).
        float max_planarity = 0.8f;         ///< Threshold to reject highly planar objects (e.g., walls).
        float min_scatter = 0.02f;          ///< Threshold to ensure the object has a volumetric shape.
    };

    /**
     * @brief Default constructor for ConeEstimator.
     */
    ConeEstimator();

    /**
     * @brief Explicit constructor with custom configuration.
     * @param config The custom configuration struct.
     */
    explicit ConeEstimator(const Config& config);

    /**
     * @brief Estimates whether a cluster is a cone using static and dynamic geometric rules.
     * 
     * The estimation process is split into two phases:
     * 1. **Phase 0: Bounding Box Early Exit**: A fast O(N) check of the axis-aligned 
     *    bounding box (AABB) to reject obviously non-cone objects before expensive 
     *    feature extraction.
     * 2. **Phase 1 & 2**: Full feature extraction and dynamic rule application.
     * 
     * @param cluster Point cloud of a single candidate object.
     * @return Cone structure with pose and binary confidence (1.0 if accepted, 0.0 if rejected).
     */
    Cone estimate(const PointCloudPtr& cluster);

    /**
     * @brief Extracts geometric and statistical features from a cluster.
     * 
     * Utilizes a **direct Eigen-based Covariance Matrix** calculation instead of 
     * iterative PCA for performance. The eigenvalues are used to derive 
     * linearity, planarity, and scattering (sphericity).
     * 
     * @param cluster Point cloud of a single candidate object.
     * @return ClusterFeatures structure containing all calculated metrics.
     */
    ClusterFeatures extractFeatures(const PointCloudPtr& cluster);

private:
    Config config_; ///< Current configuration parameters.
};

} // namespace fs_perception
