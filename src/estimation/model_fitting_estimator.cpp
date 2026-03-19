#include "estimation/model_fitting_estimator.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace fs_perception {

ModelFittingEstimator::ModelFittingEstimator() : config_(Config()) {}

ModelFittingEstimator::ModelFittingEstimator(const Config& config) : config_(config) {}

Cone ModelFittingEstimator::estimate(const PointCloudPtr& cluster) {
    Cone cone;
    cone.color = ConeColor::UNKNOWN;
    cone.confidence = 0.0f;

    // Minimum point filter for RANSAC stability
    if (cluster->size() < 10) {
        return cone;
    }

    // Phase 1: PCA Pre-filtering (Efficiency check)
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    Eigen::Vector3f eigenvalues = pca.getEigenValues().head<3>();
    float l1 = eigenvalues[0], l2 = eigenvalues[1], l3 = eigenvalues[2];
    float linearity = (l1 - l2) / l1;
    float scattering = l3 / l1;

    // Reject objects that are too linear or not sufficiently volumetric
    if (linearity > config_.max_linearity || scattering < config_.min_scatter) {
        return cone;
    }

    // Phase 2: Height and Position statistics
    const float ground_z_level = -0.52f;
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    cone.height = max_pt.z - ground_z_level;
    cone.z = min_pt.z;

    // Fast height pre-filter
    if (cone.height < config_.min_height || cone.height > config_.max_height) {
        return cone;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    cone.x = centroid[0];
    cone.y = centroid[1];
    cone.range = std::sqrt(cone.x * cone.x + cone.y * cone.y);
    cone.bearing = std::atan2(cone.y, cone.x);
    cone.cloud = cluster;

    // Phase 3: Surface Normal Estimation for Cylinder Fitting
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod(tree);
    ne.setInputCloud(cluster);
    ne.setKSearch(15); 
    ne.compute(*cloud_normals);

    // Phase 4: Geometric Cylinder Fitting using RANSAC
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(config_.normal_distance_weight);
    seg.setMaxIterations(config_.max_iterations);
    seg.setDistanceThreshold(config_.distance_threshold);
    seg.setRadiusLimits(config_.radius_min, config_.radius_max);
    seg.setInputCloud(cluster);
    seg.setInputNormals(cloud_normals);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    // Confidence assessment based on inlier percentage
    if (inliers->indices.empty()) {
        return cone;
    }

    float inlier_ratio = static_cast<float>(inliers->indices.size()) / static_cast<float>(cluster->size());

    if (inlier_ratio >= config_.min_inlier_ratio) {
        cone.confidence = inlier_ratio; 
        cone.color = ConeColor::BLUE; // Placeholder color classification
    }

    return cone;
}

} // namespace fs_perception