#ifndef FS_PERCEPTION_GROUND_REMOVER_HPP
#define FS_PERCEPTION_GROUND_REMOVER_HPP

#include "types.hpp"
#include <vector>
#include <limits> // Necessario per numeric_limits

namespace fs_perception {

// 1. Definiamo la struct qui, fuori dalla classe o pubblica dentro
struct Bin {
    float min_z = std::numeric_limits<float>::max();
    bool has_points = false;
};

class GroundRemover {
public:
    GroundRemover(); // Aggiungiamo il costruttore per inizializzare la grid
    void removeGround(const PointCloudConstPtr& cloud_in, PointCloudPtr& cloud_obstacles, PointCloudPtr& cloud_ground);

private:
    // 2. Costanti definite qui per essere usate nel sizing
    static constexpr int SEGMENTS = 100;
    static constexpr int BINS = 100;
    
    // 3. Vettore 1D (molto più veloce di vector<vector>)
    std::vector<Bin> grid_; 
};

} // namespace fs_perception

#endif