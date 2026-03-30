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

namespace fs_perception {

ConeEstimator::ConeEstimator() : config_(Config()) {}

ConeEstimator::ConeEstimator(const Config& config) : config_(config) {}

Cone ConeEstimator::estimate(const PointCloudPtr& cluster) {
    Cone cone;
    cone.color = ConeColor::UNKNOWN; 
    cone.confidence = 0.0f;

    // Minimum point filter for reliable estimation
    if (cluster->size() < 2) {
        return cone;
    }

    // Phase 1: Feature Extraction
    ClusterFeatures features = extractFeatures(cluster);
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

    // Dynamic point count threshold
    int expected_min_points = std::max(3, static_cast<int>(config_.min_points_at_10m * (100.0f / (r*r)))); 
    expected_min_points = std::min(150, expected_min_points);
    
    if (features.point_count < expected_min_points) {
        return cone;
    }

    // PCA Features (Shape)
    if (features.linearity > config_.max_linearity || 
        features.planarity > config_.max_planarity || 
        features.scattering < config_.min_scatter) {
        return cone;
    }

    // Verticality Check
    if (features.verticality < 0.6f) { 
        return cone;
    }

    // Bounding Box Rules
    float dynamic_min_width = config_.base_min_width - (r * config_.dynamic_width_decay);
    dynamic_min_width = std::max(0.02f, dynamic_min_width); 

    if (features.height < config_.min_height || features.height > config_.max_height) return cone;
    if (features.width_max < dynamic_min_width || features.width_max > config_.max_width) return cone;
    if (features.aspect_ratio < config_.min_aspect_ratio || features.aspect_ratio > config_.max_aspect_ratio) return cone;

    // Symmetry check
    if (features.symmetry > config_.max_width_diff_ratio) {
        return cone;
    }

    // Candidate accepted as a cone
    cone.confidence = 1.0f;
    cone.color = ConeColor::BLUE; 
    
    return cone;
}

ClusterFeatures ConeEstimator::extractFeatures(const PointCloudPtr& cluster) {
    ClusterFeatures f;
    f.point_count = cluster->size();

    if (f.point_count < 2) return f;

    // 1. Position and distance calculation
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    f.x = centroid[0];
    f.y = centroid[1];
    
    f.range = std::sqrt(f.x * f.x + f.y * f.y);
    f.bearing = std::atan2(f.y, f.x);

    // 2. Shape classification using PCA
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    Eigen::Vector3f eigenvalues = pca.getEigenValues().head<3>();
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    
    float l1 = eigenvalues[0];
    float l2 = eigenvalues[1];
    float l3 = eigenvalues[2];
    
    float sum_l = l1 + l2 + l3;
    if (sum_l > 1e-6) {
        f.linearity = (l1 - l2) / l1;
        f.planarity = (l2 - l3) / l1;
        f.scattering = l3 / l1;
    }
    
    f.verticality = std::abs(eigenvectors(2, 0)); 

    // 3. Intensity analysis
    double sum_intensity = 0.0;
    for (const auto& pt : cluster->points) {
        sum_intensity += pt.intensity;
    }
    f.avg_intensity = sum_intensity / f.point_count;
    
    // 4. Geometric dimensions (Bounding Box)
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    f.height = max_pt.z - config_.ground_z_level; 
    f.z = min_pt.z;

    float w_x = max_pt.x - min_pt.x;
    float w_y = max_pt.y - min_pt.y;
    f.width_max = std::max(w_x, w_y);
    f.width_min = std::min(w_x, w_y);
    
    f.aspect_ratio = (f.height > 0) ? f.width_max / f.height : 0;
    f.symmetry = (f.width_min > 0.001f) ? f.width_max / f.width_min : 10.0f;

    return f;
}

} // namespace fs_perception
