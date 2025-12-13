#include "ground_remover.hpp"
#include <cmath>
#include <algorithm>

namespace fs_perception {

// Costruttore: Riserva la memoria una volta sola all'avvio
GroundRemover::GroundRemover() {
    grid_.resize(SEGMENTS * BINS);
}

void GroundRemover::removeGround(const PointCloudConstPtr& cloud_in, PointCloudPtr& cloud_obstacles, PointCloudPtr& cloud_ground) {
    const float MAX_RANGE = 25.0f;
    const float BIN_SIZE = MAX_RANGE / BINS;
    const float SENSOR_Z = -0.52f;
    const float HARD_GROUND_CUTOFF = SENSOR_Z + 0.05f; 
    const float LOCAL_THRESHOLD = 0.04f; 

    // RESET VELOCE: Invece di ricreare il vettore, resettiamo i valori
    // Creiamo un bin "vuoto" di riferimento
    Bin empty_bin; 
    empty_bin.min_z = std::numeric_limits<float>::max();
    empty_bin.has_points = false;
    
    // std::fill è molto ottimizzato per vettori contigui
    std::fill(grid_.begin(), grid_.end(), empty_bin);

    // FASE 1: Trova il minimo locale (LPR)
    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        if (r < 0.5f || r > MAX_RANGE) continue;

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / SEGMENTS)) % SEGMENTS;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= BINS) b_idx = BINS - 1;

        // --- CORREZIONE ACCESSO 1D ---
        // Indice = Riga * Larghezza + Colonna
        int idx = s_idx * BINS + b_idx; 

        if (pt.z < grid_[idx].min_z) {
            grid_[idx].min_z = pt.z;
            grid_[idx].has_points = true;
        }
    }

    // FASE 2: Classificazione
    // Ottimizzazione: Riserva memoria per evitare riallocazioni durante il push_back
    cloud_obstacles->points.reserve(cloud_in->points.size() / 2);
    cloud_ground->points.reserve(cloud_in->points.size() / 2);

    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y); // Qui potresti usare r_squared per evitare sqrt, ma serve r per il binning
        if (r < 0.5f || r > MAX_RANGE) continue;

        if (pt.z < HARD_GROUND_CUTOFF) {
            cloud_ground->push_back(pt);
            continue;
        }

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / SEGMENTS)) % SEGMENTS;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= BINS) b_idx = BINS - 1;

        // --- CORREZIONE ACCESSO 1D ---
        int idx = s_idx * BINS + b_idx;

        bool is_obstacle = false;

        if (grid_[idx].has_points) {
            float local_ground = grid_[idx].min_z;
            if (pt.z > (local_ground + LOCAL_THRESHOLD)) {
                is_obstacle = true;
            }
        } else {
            is_obstacle = true;
        }

        if (is_obstacle) {
            cloud_obstacles->push_back(pt);
        } else {
            cloud_ground->push_back(pt);
        }
    }
}

}