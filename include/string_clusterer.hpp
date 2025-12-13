#pragma once

// NOTA: Include diretto senza cartella
#include "types.hpp"
#include <vector>

namespace fs_perception {

class StringClusterer {
public:
    /**
     * @brief Raggruppa i punti ostacolo in oggetti distinti.
     * Sfrutta l'ordinamento spaziale/temporale dei punti per evitare KD-Tree (O(N)).
     * * @param cloud Input: Nuvola ostacoli (senza terreno)
     * @param clusters Output: Vettore di nuvole, dove ogni nuvola è un cluster (cono candidato)
     */
    void cluster(const PointCloudPtr& cloud, 
                 std::vector<PointCloudPtr>& clusters);
};

} // namespace fs_perception
