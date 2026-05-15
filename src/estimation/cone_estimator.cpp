#include "estimation/cone_estimator.hpp"
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

namespace lidar_perception {

ConeEstimator::ConeEstimator() : config_(Config()) {}

ConeEstimator::ConeEstimator(const Config& config) : config_(config) {}

Cone ConeEstimator::estimate(const PointCloudPtr& cluster) {
    Cone cone;
    cone.color = ConeColor::UNKNOWN; 
    cone.confidence = 0.0f;

    if (cluster->size() < 2) return cone;

    // Phase 1: Feature Extraction (Optimized single-pass extraction)
    ClusterFeatures features = extractFeatures(cluster);
    
    // Phase 0: Bounding Box Early Exit (Fast O(1) after extraction)
    float h = features.height + (features.z - config_.ground_z_level);
    float w = features.width_max;

    if (h < config_.min_height * 0.8f || h > config_.max_height * 1.2f || w > config_.max_width * 1.5f) {
        cone.features = features;
        cone.features.rejection_reason = "EARLY_AABB";
        return cone;
    }

    cone.x = features.x;
    cone.y = features.y;
    cone.z = features.z;
    cone.range = features.range;
    cone.bearing = features.bearing;
    cone.height = features.height;
    cone.cloud = cluster;
    cone.features = features;

    // Phase 2: Dynamic Thresholding and Classification
    float r = features.range;

    // Dynamic point count threshold - Adjusted for 2.0cm Voxel Grid and 40-channel LiDAR
    // At 10m, 5 points are enough. At 2m, we cap at config_.min_points_cap (physical limit)
    int expected_min_points = std::max(3, static_cast<int>(config_.min_points_at_10m * (100.0f / (r*r)))); 
    expected_min_points = std::min(config_.min_points_cap, expected_min_points); 
    
    // Soft-Pass Logic: If the shape is exceptionally vertical and symmetric, 
    // allow a 15% deficit in point count.
    float point_pass_ratio = (float)features.point_count / (float)expected_min_points;
    bool high_quality_shape = (features.verticality > 0.85f && features.symmetry < 1.5f);

    if (point_pass_ratio < 0.85f && !high_quality_shape) {
        cone.features.rejection_reason = "POINT_COUNT";
        return cone;
    }

    // Intensity Filter
    if (features.avg_intensity < config_.min_intensity) {
        cone.features.rejection_reason = "LOW_INTENSITY";
        return cone;
    }

    // PCA Features (Shape) - Relaxed thresholds for better stability
    if (features.linearity > (config_.max_linearity)) {
        cone.features.rejection_reason = "PCA_LINEARITY";
        return cone;
    }
    if (features.planarity > (config_.max_planarity + 0.1f)) {
        cone.features.rejection_reason = "PCA_PLANARITY";
        return cone;
    }
    if (features.scattering < (config_.min_scatter * 0.3f)) {
        cone.features.rejection_reason = "PCA_SCATTER";
        return cone;
    }
 
    // Verticality Check
    if (features.verticality < config_.min_verticality) { 
        cone.features.rejection_reason = "VERTICALITY";
        return cone;
    }

    // Bounding Box Rules
    float dynamic_min_width = config_.base_min_width - (r * config_.dynamic_width_decay);
    dynamic_min_width = std::max(0.02f, dynamic_min_width); 

    if (features.height < config_.min_height || features.height > config_.max_height) {
        cone.features.rejection_reason = "HEIGHT_LIMIT";
        return cone;
    }
    if (features.width_max < dynamic_min_width || features.width_max > config_.max_width) {
        cone.features.rejection_reason = "WIDTH_LIMIT";
        return cone;
    }
    if (features.aspect_ratio < config_.min_aspect_ratio || features.aspect_ratio > config_.max_aspect_ratio) {
        cone.features.rejection_reason = "ASPECT_RATIO";
        return cone;
    }

    // Symmetry check
    if (features.symmetry > config_.max_width_diff_ratio) {
        cone.features.rejection_reason = "SYMMETRY";
        return cone;
    }

    // Candidate accepted as a cone
    cone.confidence = 1.0f;
    cone.features.confidence = 1.0f;
    cone.features.rejection_reason = "NONE";
    cone.color = ConeColor::UNKNOWN;
    
    return cone;
}

ClusterFeatures ConeEstimator::extractFeatures(const PointCloudPtr& cluster) {
    ClusterFeatures f;
    f.point_count = cluster->size();
    if (f.point_count < 2) return f;

    // 1. Single pass for Min/Max and Intensity
    PointT min_pt, max_pt;
    min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
    max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();
    double sum_intensity = 0.0;

    for (const auto& pt : cluster->points) {
        if (pt.x < min_pt.x) min_pt.x = pt.x;
        if (pt.y < min_pt.y) min_pt.y = pt.y;
        if (pt.z < min_pt.z) min_pt.z = pt.z;
        if (pt.x > max_pt.x) max_pt.x = pt.x;
        if (pt.y > max_pt.y) max_pt.y = pt.y;
        if (pt.z > max_pt.z) max_pt.z = pt.z;
        sum_intensity += pt.intensity;
    }

    f.avg_intensity = static_cast<float>(sum_intensity / f.point_count);
    f.height = max_pt.z - min_pt.z; 
    f.z = min_pt.z;
    float w_x = max_pt.x - min_pt.x;
    float w_y = max_pt.y - min_pt.y;
    f.width_max = std::max(w_x, w_y);
    f.width_min = std::max(0.001f, std::min(w_x, w_y));
    f.aspect_ratio = (f.height > 0.001f) ? f.width_max / f.height : 0.0f;
    f.symmetry = f.width_max / f.width_min;

    // 2. Covariance and Mean in one pass
    Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f mean;
    pcl::computeMeanAndCovarianceMatrix(*cluster, covariance_matrix, mean);

    f.x = mean[0];
    f.y = mean[1];
    f.range = std::sqrt(f.x * f.x + f.y * f.y);
    f.bearing = std::atan2(f.y, f.x);

    // 3. Shape classification using Eigen solver
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix);
    Eigen::Vector3f eigenvalues = solver.eigenvalues(); 
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();

    float l1 = eigenvalues[2];
    float l2 = eigenvalues[1];
    float l3 = eigenvalues[0];
    
    float sum_l = l1 + l2 + l3;
    if (sum_l > 1e-6f) {
        f.linearity = (l1 - l2) / l1;
        f.planarity = (l2 - l3) / l1;
        f.scattering = l3 / l1;
    }
    f.verticality = std::abs(eigenvectors(2, 2)); 

    return f;
}

} // namespace lidar_perception
