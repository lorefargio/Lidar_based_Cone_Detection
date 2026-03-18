#pragma once

#include "estimation/estimator_interface.hpp"

namespace fs_perception {

class ModelFittingEstimator : public EstimatorInterface {
public:
    struct Config {
        float normal_distance_weight = 0.1f;
        int max_iterations = 1000;
        float distance_threshold = 0.05f;
        float radius_min = 0.05f;
        float radius_max = 0.18f; // cone standard radius is ~0.114m
        float min_inlier_ratio = 0.5f; // At least 50% of points should be inliers
        float min_height = 0.15f;
        float max_height = 0.35f;
    };

    ModelFittingEstimator();
    explicit ModelFittingEstimator(const Config& config);

    /**
     * @brief Stima se il cluster è un cono utilizzando RANSAC per fittare un cilindro.
     * @param cluster Punti di un singolo oggetto candidato
     * @return Cone Struttura contenente posizione (XYZ) e confidenza
     */
    Cone estimate(const PointCloudPtr& cluster) override;

private:
    Config config_;
};

} // namespace fs_perception
