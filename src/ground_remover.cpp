#include "ground_remover.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace fs_perception {

void GroundRemover::removeGround(const PointCloudConstPtr& cloud_in, PointCloudPtr& cloud_obstacles, PointCloudPtr& cloud_ground) {
    // PARAMETRI OTTIMIZZATI PER RECUPERARE LA BASE DEI CONI
    const int SEGMENTS = 100;
    const int BINS = 100;
    const float MAX_RANGE = 25.0f;
    const float BIN_SIZE = MAX_RANGE / BINS;
    
    // TUA MISURA DEL SUOLO: -0.52m
    const float SENSOR_Z = -0.52f;

    // 1. HARD CUT (Taglio Assoluto)
    // Prima era +0.10. Ora mettiamo +0.05 (5cm).
    // Significa: tutto ciò che è sotto -0.47m viene scartato come asfalto.
    // La base del cono parte da -0.52, quindi perdiamo solo i primi 5cm (accettabile).
    const float HARD_GROUND_CUTOFF = SENSOR_Z + 0.05f; 

    // 2. SOGLIA DINAMICA (Rispetto al minimo locale)
    // Se un punto è anche solo 5cm sopra il punto più basso vicino, è un ostacolo.
    // Questo permette di vedere la "pancia" del cono e non solo la punta.
    const float LOCAL_THRESHOLD = 0.04f; 

    struct Bin {
        float min_z = std::numeric_limits<float>::max();
        bool has_points = false;
    };

    std::vector<std::vector<Bin>> grid(SEGMENTS, std::vector<Bin>(BINS));

    // FASE 1: Trova il minimo locale (LPR)
    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        
        // Accettiamo punti anche leggermente sotto il livello teorico del suolo per il calcolo del minimo
        // (es. buche o errori di calibrazione)
        if (r < 0.5f || r > MAX_RANGE) continue;

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / SEGMENTS)) % SEGMENTS;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= BINS) b_idx = BINS - 1;

        if (pt.z < grid[s_idx][b_idx].min_z) {
            grid[s_idx][b_idx].min_z = pt.z;
            grid[s_idx][b_idx].has_points = true;
        }
    }

    // FASE 2: Classificazione
    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        if (r < 0.5f || r > MAX_RANGE) continue;

        // Se è sotto il livello assoluto dell'asfalto (+ tolleranza 5cm), è ground.
        if (pt.z < HARD_GROUND_CUTOFF) {
            cloud_ground->push_back(pt);
            continue;
        }

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / SEGMENTS)) % SEGMENTS;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= BINS) b_idx = BINS - 1;

        bool is_obstacle = false;

        if (grid[s_idx][b_idx].has_points) {
            float local_ground = grid[s_idx][b_idx].min_z;
            
            // Logica Ibrida:
            // O supera la soglia locale (es. dosso)
            // O è molto alto in assoluto (es. ostacolo sopra una buca)
            if (pt.z > (local_ground + LOCAL_THRESHOLD)) {
                is_obstacle = true;
            }
        } else {
            // Se non abbiamo riferimenti locali ma è sopra l'hard cutoff, lo teniamo
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
