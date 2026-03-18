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

    // Filtro base punti (almeno 2 per calcoli successivi)
    if (cluster->size() < 2) return cone;

    // --- CALCOLO STATISTICHE E POSIZIONE ---
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    cone.x = centroid[0];
    cone.y = centroid[1];
    
    // 1. Calcolo Distanza (r) per Soglie Dinamiche
    float r = std::sqrt(cone.x * cone.x + cone.y * cone.y);

    // Calcolo minimo numero di punti attesi in base alla distanza (Più realistico del precedente)
    // Formula: expected = points_at_10m * (10 / r)^1.5 -> Calo meno brusco del r^2
    int expected_min_points = std::max(3, static_cast<int>(config_.min_points_at_10m * std::pow(10.0f / r, 1.5f))); 
    
    // Filtro Punti Dinamico (con limite superiore per i vicini per non essere troppo rigido)
    int max_expected_at_near = 150; 
    expected_min_points = std::min(expected_min_points, max_expected_at_near);

    if (cluster->size() < static_cast<size_t>(expected_min_points)) {
        return cone;
    }

    // --- ANALISI PCA (Shape Classifier) ---
    pcl::PCA<PointT> pca;
    pca.setInputCloud(cluster);
    Eigen::Vector3f eigenvalues = pca.getEigenValues().head<3>();
    
    float l1 = eigenvalues[0];
    float l2 = eigenvalues[1];
    float l3 = eigenvalues[2];
    
    // Normalizziamo la somma degli autovalori (Varianza Totale)
    float sum_l = l1 + l2 + l3;
    if (sum_l < 1e-6) return cone;
    
    // Calcoliamo Linearity (L), Planarity (P), Scattering (S)
    float linearity = (l1 - l2) / l1;
    float planarity = (l2 - l3) / l1;
    float scattering = l3 / l1;

    // Filtriamo in base alla forma (State-of-the-Art per scartare gambe/muri)
    if (linearity > config_.max_linearity || planarity > config_.max_planarity || scattering < config_.min_scatter) {
        return cone;
    }

    double sum_intensity = 0.0;
    for (const auto& pt : cluster->points) sum_intensity += pt.intensity;
    double avg_intensity = sum_intensity / cluster->size();
    
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    cone.height = max_pt.z - ground_z_level; 
    cone.z = min_pt.z;

    // Dimensioni
    float w_x = max_pt.x - min_pt.x;
    float w_y = max_pt.y - min_pt.y;
    float max_width = std::max(w_x, w_y);
    float min_width = std::min(w_x, w_y);
    
    float aspect_ratio = (cone.height > 0) ? max_width / cone.height : 0;

    // 2. Soglia Dinamica Larghezza Minima
    float dynamic_min_width = config_.base_min_width - (r * config_.dynamic_width_decay);
    dynamic_min_width = std::max(0.02f, dynamic_min_width); // Limite inferiore di sicurezza

    // 3. INTENSITÀ:
    if (avg_intensity < config_.min_intensity) { 
        return cone; 
    }

    // 4. ALTEZZA:
    if (cone.height < config_.min_height || cone.height > config_.max_height) {
        return cone;
    }

    // 5. LARGHEZZA (Con soglia minima dinamica):
    if (max_width < dynamic_min_width || max_width > config_.max_width) {
        return cone;
    }
    
    // 6. ASPECT RATIO:
    if (aspect_ratio < config_.min_aspect_ratio || aspect_ratio > config_.max_aspect_ratio) {
        return cone;
    }

    // 7. LINEARITY (WIDTH DIFFERENCE):
    if (max_width > (min_width * config_.max_width_diff_ratio)) {
        return cone;
    }

    // --- SUCCESSO ---
    cone.confidence = 1.0f;
    cone.color = ConeColor::BLUE; // Colore fisso per vederli tutti
    
    return cone;
}

} // namespace fs_perception