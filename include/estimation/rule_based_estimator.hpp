#pragma once

#include "estimation/estimator_interface.hpp"

namespace fs_perception {

class RuleBasedEstimator : public EstimatorInterface {
public:
    struct Config {
        float dynamic_width_decay = 0.005f; // decremento della min_width per metro
        int min_points_at_10m = 10;         // numero minimo di punti attesi a 10 metri
        
        float min_height = 0.10f;
        float max_height = 0.50f;
        float base_min_width = 0.8f;
        float max_width = 0.36f;
        float min_aspect_ratio = 0.4f;
        float max_aspect_ratio = 1.2f;
        float max_width_diff_ratio = 2.5f;
        float min_intensity = 5.0f;

        // Analisi PCA (Principal Component Analysis)
        float max_linearity = 0.8f;   // Scarta oggetti lineari (paletti, gambe)
        float max_planarity = 0.8f;   // Scarta oggetti piatti (pezzi di muro)
        float min_scatter = 0.05f;    // Assicura che l'oggetto sia volumetrico
    };

    RuleBasedEstimator();
    explicit RuleBasedEstimator(const Config& config);

    /**
     * @brief Stima posizione e colore di un cluster basandosi su regole geometriche statiche.
     * @param cluster Punti di un singolo oggetto candidato
     * @return Cone Struttura contenente posizione (XYZ) e confidenza
     */
    Cone estimate(const PointCloudPtr& cluster) override;

private:
    Config config_;
};

} // namespace fs_perception