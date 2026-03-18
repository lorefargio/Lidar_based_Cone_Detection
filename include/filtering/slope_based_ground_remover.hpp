#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <vector>
#include <cmath>

namespace fs_perception {

/**
 * @brief Algoritmo di Ground Removal basato sulla pendenza radiale (Slope-based).
 * Risulta più accurato nel preservare la base dei coni rispetto all'approccio a soglia fissa.
 */
class SlopeBasedGroundRemover : public GroundRemoverInterface {
public:
    struct Config {
        float max_range = 25.0f;
        float sensor_z = -0.52f;
        float max_slope = 0.08f;          // Pendenza massima tollerata per il terreno (circa 4.5 gradi)
        float max_z_diff = 0.05f;         // Differenza massima di altezza tra punti consecutivi
        float initial_ground_threshold = 0.05f; // Soglia iniziale vicino al sensore
        int segments = 360;               // Risoluzione angolare elevata per precisione
    };

    SlopeBasedGroundRemover();
    explicit SlopeBasedGroundRemover(const Config& config);

    void removeGround(const PointCloudConstPtr& cloud_in, 
                      PointCloudPtr& cloud_obstacles, 
                      PointCloudPtr& cloud_ground) override;

private:
    Config config_;
};

} // namespace fs_perception
