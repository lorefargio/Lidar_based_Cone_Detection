#pragma once

// NOTA: Include diretto senza cartella
#include "types.hpp" 

namespace fs_perception {

class GroundRemover {
public:
    /**
     * @brief Rimuove il terreno usando "Lowest Point Representative" e segmentazione polare.
     * Ottimizzato per Pandar40P sul musetto (gestisce bene angoli radenti).
     * * @param cloud_in Input: Nuvola grezza dal LiDAR
     * @param cloud_obstacles Output: Punti che NON sono terreno (coni potenziali)
     * @param cloud_ground Output: Punti classificati come terreno (per debug in RViz)
     */
    void removeGround(const PointCloudConstPtr& cloud_in, 
                      PointCloudPtr& cloud_obstacles, 
                      PointCloudPtr& cloud_ground);
};

} // namespace fs_perception
