#pragma once

#include "estimation/estimator_interface.hpp"

namespace fs_perception {

class RuleBasedEstimator : public EstimatorInterface {
public:
    struct Config {
        float dynamic_width_decay = 0.005f; // decremento della min_width per metro
        int min_points_at_10m = 10;         // numero minimo di punti attesi a 10 metri
        
        float min_height = 0.15f;
        float max_height = 0.35f;
        float base_min_width = 0.10f;
        float max_width = 0.26f;
        float min_aspect_ratio = 0.4f;
        float max_aspect_ratio = 1.2f;
        float max_width_diff_ratio = 2.5f;
        float min_intensity = 5.0f;
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