#include "estimation/model_fitting_estimator.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
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

    // Filtro base punti: RANSAC ha bisogno di un minimo di punti per fittare bene
    if (cluster->size() < 10) return cone;

    // --- CALCOLO STATISTICHE E POSIZIONE ---
    const float ground_z_level = -0.52f;
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    cone.height = max_pt.z - ground_z_level;
    cone.z = min_pt.z;

    // 1. ALTEZZA (Pre-filtro veloce)
    if (cone.height < config_.min_height || cone.height > config_.max_height) {
        return cone;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    cone.x = centroid[0];
    cone.y = centroid[1];

    // 2. STIMA DELLE NORMALI
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    ne.setSearchMethod(tree);
    ne.setInputCloud(cluster);
    ne.setKSearch(10); // Usa 10 vicini per stimare le normali
    ne.compute(*cloud_normals);

    // 3. FITTING DEL CILINDRO CON RANSAC
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

    if (inliers->indices.empty()) {
        return cone;
    }

    // 4. VALUTAZIONE CONFIDENZA
    float inlier_ratio = static_cast<float>(inliers->indices.size()) / static_cast<float>(cluster->size());

    if (inlier_ratio >= config_.min_inlier_ratio) {
        cone.confidence = inlier_ratio; // Confidenza basata sulla percentuale di inliers
        cone.color = ConeColor::BLUE; // Colore di default per visualizzazione
    }

    return cone;
}

} // namespace fs_perception
