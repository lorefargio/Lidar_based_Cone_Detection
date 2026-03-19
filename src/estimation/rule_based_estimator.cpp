#include "estimation/rule_based_estimator.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream> 
#include <rclcpp/rclcpp.hpp>

namespace fs_perception {

RuleBasedEstimator::RuleBasedEstimator() : config_(Config()) {}

RuleBasedEstimator::RuleBasedEstimator(const Config& config) : config_(config) {}

Cone RuleBasedEstimator::estimate(const PointCloudPtr& cluster) {
    const float ground_z_level = -0.52f;

    Cone cone;
    cone.color = ConeColor::UNKNOWN; 
    cone.confidence = 0.0f;

    // Minimum point filter for reliable estimation
    if (cluster->size() < 2) {
        return cone;
    }

    // Phase 1: Position and distance calculation
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    cone.x = centroid[0];
    cone.y = centroid[1];
    
    // Radial distance and bearing calculation
    float r = std::sqrt(cone.x * cone.x + cone.y * cone.y);
    cone.range = r;
    cone.bearing = std::atan2(cone.y, cone.x);
    cone.cloud = cluster;

    // Dynamic point count threshold (Inversely proportional to distance square)
    // Capped at 150 points for very close ranges
    int expected_min_points = std::max(3, static_cast<int>(config_.min_points_at_10m * (100.0f / (r*r)))); 
    expected_min_points = std::min(150, expected_min_points);
    
    if (cluster->size() < static_cast<size_t>(expected_min_points)) {
        return cone;
    }

    // Phase 2: Shape classification using Principal Component Analysis (PCA)
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    Eigen::Vector3f eigenvalues = pca.getEigenValues().head<3>();
    
    float l1 = eigenvalues[0];
    float l2 = eigenvalues[1];
    float l3 = eigenvalues[2];
    
    // Total variance normalization
    float sum_l = l1 + l2 + l3;
    if (sum_l < 1e-6) return cone;
    
    // Features: Linearity (L), Planarity (P), Scattering (S)
    float linearity = (l1 - l2) / l1;
    float planarity = (l2 - l3) / l1;
    float scattering = l3 / l1;

    // Shape-based filter to reject poles, legs, and walls
    if (linearity > config_.max_linearity || planarity > config_.max_planarity || scattering < config_.min_scatter) {
        return cone;
    }

    // Phase 3: Intensity analysis
    double sum_intensity = 0.0;
    for (const auto& pt : cluster->points) {
        sum_intensity += pt.intensity;
    }
    double avg_intensity = sum_intensity / cluster->size();
    
    // Phase 4: Geometric rule evaluation (Bounding Box)
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    cone.height = max_pt.z - ground_z_level; 
    cone.z = min_pt.z;

    // Horizontal dimensions
    float w_x = max_pt.x - min_pt.x;
    float w_y = max_pt.y - min_pt.y;
    float max_width = std::max(w_x, w_y);
    float min_width = std::min(w_x, w_y);
    
    float aspect_ratio = (cone.height > 0) ? max_width / cone.height : 0;

    // Distance-based dynamic width minimum threshold
    float dynamic_min_width = config_.base_min_width - (r * config_.dynamic_width_decay);
    dynamic_min_width = std::max(0.02f, dynamic_min_width); // Safety lower bound

    // Apply rule thresholds
    if (avg_intensity < config_.min_intensity) return cone; 
    if (cone.height < config_.min_height || cone.height > config_.max_height) return cone;
    if (max_width < dynamic_min_width || max_width > config_.max_width) return cone;
    if (aspect_ratio < config_.min_aspect_ratio || aspect_ratio > config_.max_aspect_ratio) return cone;

    // Symmetry check (Reject highly elongated objects)
    if (max_width > (min_width * config_.max_width_diff_ratio)) {
        return cone;
    }

    // Candidate accepted as a cone
    cone.confidence = 1.0f;
    cone.color = ConeColor::BLUE; // Placeholder color classification
    
    return cone;
}

} // namespace fs_perception
