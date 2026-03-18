#include "clustering/string_clusterer.hpp"
#include <cmath>

namespace fs_perception {

void StringClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    // Parametri
    const float MAX_DIST_ = 0.3f; // Distanza massima tra punti dello stesso cono 
    const int MIN_CLUSTER_SIZE = 3; // Pandar40P ha alta densità
    const int MAX_CLUSTER_SIZE = 300;
    const float MAX_INT_JUMP = 100.0f ; 

    // IMPORTANTE: Assumiamo che il driver Hesai pubblichi i punti ordinati per Ring/Firing sequence.
    // Se la bag è "unordered", questo algoritmo degrada.
    // L'algoritmo itera linearmente[cite: 390].

    PointCloudPtr current_cluster(new PointCloud);
    current_cluster->push_back(cloud->points[0]);

    for (size_t i = 1; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        const auto& prev = cloud->points[i-1];

        // Calcola il raggio (distanza dal sensore)
        float range = std::sqrt(pt.x*pt.x + pt.y*pt.y);

        float dynamic_threshold = MAX_DIST_ + (range * 0.02f) ;
        float max_dist_sq = dynamic_threshold * dynamic_threshold ;

        // Calcolo distanza rapida (senza radice quadrata)
        float dist_sq = (pt.x - prev.x)*(pt.x - prev.x) + 
                        (pt.y - prev.y)*(pt.y - prev.y) + 
                        (pt.z - prev.z)*(pt.z - prev.z);

        float int_diff = std::abs(pt.intensity -prev.intensity) ;
        bool intensity_jump = int_diff > MAX_INT_JUMP ;
        
        // Logica String: Se vicino al precedente, fa parte dello stesso oggetto (o "stringa")
        if (dist_sq < max_dist_sq && !intensity_jump) {
            current_cluster->push_back(pt);
        } else {
            // "Taglio" della stringa. Salva cluster se valido.
            if (current_cluster->size() >= MIN_CLUSTER_SIZE && current_cluster->size() <= MAX_CLUSTER_SIZE) {
                clusters.push_back(current_cluster);
            }
            // Reset nuovo cluster
            current_cluster.reset(new PointCloud);
            current_cluster->push_back(pt);
        }
    }
    // Aggiungi l'ultimo
    if (current_cluster->size() >= MIN_CLUSTER_SIZE) {
        clusters.push_back(current_cluster);
    }
}

}
