#pragma once

#include "utils/types.hpp"

namespace fs_perception {

/**
 * @brief Interfaccia comune per la rimozione del suolo dalla point cloud.
 * Permette di scambiare algoritmi a runtime per migliorare l'accuratezza del rilevamento dei coni.
 */
class GroundRemoverInterface {
public:
    virtual ~GroundRemoverInterface() = default;

    /**
     * @brief Divide la nuvola in punti ostacolo e punti terreno.
     * @param cloud_in Nuvola di input filtrata (non-NaN e range limitato)
     * @param cloud_obstacles Nuvola di output contenente gli oggetti (coni, marciapiedi, ecc.)
     * @param cloud_ground Nuvola di output contenente i punti del suolo (per debug)
     */
    virtual void removeGround(const PointCloudConstPtr& cloud_in, 
                              PointCloudPtr& cloud_obstacles, 
                              PointCloudPtr& cloud_ground) = 0;
};

} // namespace fs_perception
