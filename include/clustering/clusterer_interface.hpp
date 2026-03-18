#pragma once

#include "utils/types.hpp"
#include <vector>

namespace fs_perception {

class ClustererInterface {
public:
    virtual ~ClustererInterface() = default;

    /**
     * @brief Raggruppa i punti ostacolo in oggetti distinti.
     * @param cloud Input: Nuvola ostacoli (senza terreno)
     * @param clusters Output: Vettore di nuvole, dove ogni nuvola è un cluster (cono candidato)
     */
    virtual void cluster(const PointCloudPtr& cloud, 
                         std::vector<PointCloudPtr>& clusters) = 0;
};

} // namespace fs_perception