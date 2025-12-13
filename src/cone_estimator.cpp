#include "cone_estimator.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream> 
#include <rclcpp/rclcpp.hpp> // Per i log nel terminale

namespace fs_perception {

Cone ConeEstimator::estimate(const PointCloudPtr& cluster) {
    Cone cone;
    cone.color = ConeColor::UNKNOWN; 
    cone.confidence = 0.0f;

    // Filtro base punti (questo lo teniamo)
    if (cluster->size() < 2) return cone;

    // --- CALCOLO STATISTICHE ---
    double sum_intensity = 0.0;
    for (const auto& pt : cluster->points) sum_intensity += pt.intensity;
    double avg_intensity = sum_intensity / cluster->size();

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    cone.x = centroid[0];
    cone.y = centroid[1];
    
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    // Altezza corretta rispetto al terreno noto
    float ground_z_level = -0.52f;
    cone.height = max_pt.z - ground_z_level; 
    cone.z = min_pt.z;

    // Dimensioni
    float w_x = max_pt.x - min_pt.x;
    float w_y = max_pt.y - min_pt.y;
    float max_width = std::max(w_x, w_y);
    float min_width = std::min(w_x, w_y);
    
    float aspect_ratio = (cone.height > 0) ? max_width / cone.height : 0;


    // 1. INTENSITÀ: 
    if (avg_intensity < 5.0) { 
        // std::cout << "  -> Scartato per Intensità (" << avg_intensity << ")" << std::endl;
        return cone; 
    }

    // 2. ALTEZZA: 
    if (cone.height < 0.06f || cone.height > 0.40f) {
        // std::cout << "  -> Scartato per Altezza Min (" << cone.height << ")" << std::endl;
        return cone;
    }

    // 3. LARGHEZZA:
    if (max_width < 0.04f || max_width > 0.30f) {
        return cone;
    }
    
    // 4. ASPECT RATIO:
    
    if (aspect_ratio > 2.0f) {
        //std::cout << "  -> Scartato per Ratio (" << aspect_ratio << ")" << std::endl;
        return cone;
    }

    // --- SUCCESSO ---
    cone.confidence = 1.0f;
    cone.color = ConeColor::BLUE; // Colore fisso per vederli tutti

    // Logga successo per capire i valori dei coni VERI
    /*
    if (cone.x < 10.0) {
        RCLCPP_INFO(rclcpp::get_logger("cone_est"), 
            "DETECTED! Int: %.1f | H: %.2f | W: %.2f | R: %.2f", avg_intensity, cone.height, max_width,aspect_ratio);
    }*/
    
    return cone;
}

}