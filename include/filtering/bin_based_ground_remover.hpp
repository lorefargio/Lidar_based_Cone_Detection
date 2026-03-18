#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <vector>
#include <limits>

namespace fs_perception {

/**
 * @brief Implementazione base del Ground Removal basata su griglia polare e soglia di altezza locale.
 * Sostituisce la vecchia classe GroundRemover.
 */
class BinBasedGroundRemover : public GroundRemoverInterface {
public:
    struct Config {
        float max_range = 25.0f;
        float sensor_z = -0.52f;
        float hard_ground_cutoff = -0.47f; // sensor_z + 0.05
        float local_threshold = 0.04f;      // 4cm sopra il minimo locale
        int segments = 100;
        int bins = 100;
    };

    BinBasedGroundRemover();
    explicit BinBasedGroundRemover(const Config& config);

    void removeGround(const PointCloudConstPtr& cloud_in, 
                      PointCloudPtr& cloud_obstacles, 
                      PointCloudPtr& cloud_ground) override;

private:
    struct Bin {
        float min_z = std::numeric_limits<float>::max();
        bool has_points = false;
    };

    Config config_;
    std::vector<Bin> grid_; 
};

} // namespace fs_perception
