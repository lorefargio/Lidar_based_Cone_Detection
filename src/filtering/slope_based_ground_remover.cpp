#include "filtering/slope_based_ground_remover.hpp"
#include <algorithm>
#include <vector>

namespace fs_perception {

SlopeBasedGroundRemover::SlopeBasedGroundRemover() : config_(Config()) {}

SlopeBasedGroundRemover::SlopeBasedGroundRemover(const Config& config) : config_(config) {}

void SlopeBasedGroundRemover::removeGround(const PointCloudConstPtr& cloud_in, PointCloudPtr& cloud_obstacles, PointCloudPtr& cloud_ground) {
    if (cloud_in->empty()) return;

    // 1. Organizziamo i punti per settori angolari
    std::vector<std::vector<int>> sectors(config_.segments);
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        const auto& pt = cloud_in->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / config_.segments)) % config_.segments;
        sectors[s_idx].push_back(i);
    }

    // Riserviamo memoria per performance
    cloud_obstacles->points.reserve(cloud_in->size());
    cloud_ground->points.reserve(cloud_in->size());

    // 2. Per ogni settore, analizziamo i punti in ordine di distanza radiale
    for (int s = 0; s < config_.segments; ++s) {
        auto& sector_indices = sectors[s];
        if (sector_indices.empty()) continue;

        // Ordiniamo i punti del settore per raggio (r = sqrt(x^2 + y^2))
        std::sort(sector_indices.begin(), sector_indices.end(), [&](int a, int b) {
            const auto& pt_a = cloud_in->points[a];
            const auto& pt_b = cloud_in->points[b];
            return (pt_a.x * pt_a.x + pt_a.y * pt_a.y) < (pt_b.x * pt_b.x + pt_b.y * pt_b.y);
        });

        float last_ground_r = 0.0f;
        float last_ground_z = config_.sensor_z;

        for (int idx : sector_indices) {
            const auto& pt = cloud_in->points[idx];
            float current_r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            
            if (current_r < 0.5f) {
                 cloud_ground->push_back(pt);
                 continue;
            }

            float dr = current_r - last_ground_r;
            float dz = std::abs(pt.z - last_ground_z);

            bool is_ground = false;

            // Logica pendenza: se la differenza di altezza rispetto alla distanza è piccola
            if (dr > 0.05f) {
                float slope = dz / dr;
                if (slope < config_.max_slope && dz < config_.max_z_diff) {
                    is_ground = true;
                }
            } else {
                // Se i punti sono troppo vicini radialmente (stesso anello LiDAR o quasi),
                // controlliamo solo l'altezza assoluta rispetto all'ultimo suolo noto
                if (dz < config_.max_z_diff) {
                    is_ground = true;
                }
            }

            // Sicurezza vicino al muso dell'auto (hard cutoff)
            if (pt.z < (config_.sensor_z + config_.initial_ground_threshold) && current_r < 3.0f) {
                is_ground = true;
            }

            if (is_ground) {
                cloud_ground->push_back(pt);
                last_ground_r = current_r;
                last_ground_z = pt.z;
            } else {
                cloud_obstacles->push_back(pt);
            }
        }
    }
}

} // namespace fs_perception
