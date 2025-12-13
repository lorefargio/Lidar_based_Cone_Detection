#pragma once

// NOTA: Include diretto senza cartella
#include "types.hpp"

namespace fs_perception {

class ConeEstimator {
public:
    /**
     * @brief Stima posizione e colore di un cluster.
     * Usa Ordinary Least Squares (OLS) sull'intensità per distinguere Giallo/Blu.
     * * @param cluster Punti di un singolo oggetto candidato
     * @return Cone Struttura contenente posizione (XYZ) e colore stimato
     */
    Cone estimate(const PointCloudPtr& cluster);
};

} // namespace fs_perception
