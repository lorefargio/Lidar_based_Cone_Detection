#pragma once
#include "utils/types.hpp"

namespace fs_perception {

/**
 * @class EstimatorInterface
 * @brief Common interface for all cone estimation algorithms.
 * 
 * Defines the standard structure for classifying cluster objects as cones 
 * and estimating their properties (XYZ position, color, and confidence).
 */
class EstimatorInterface {
public:
    virtual ~EstimatorInterface() = default;
    
    /**
     * @brief Estimates whether the cluster is a cone and evaluates its properties.
     * @param cluster Point cloud containing the candidate object's points.
     * @return Cone structure containing the confidence level (0.0 to 1.0) and pose.
     */
    virtual Cone estimate(const PointCloudPtr& cluster) = 0;
};

} // namespace fs_perception
