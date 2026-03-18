#include "clustering/grid_clusterer.hpp"
#include <queue>
#include <cmath>

namespace fs_perception {

GridClusterer::GridClusterer(float grid_resolution, float max_range, int min_cluster_size, int max_cluster_size)
    : grid_res_(grid_resolution), max_range_(max_range), min_size_(min_cluster_size), max_size_(max_cluster_size) {}

void GridClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    // Mappa che rappresenta la nostra griglia 2D sparsa
    std::unordered_map<std::pair<int, int>, Cell, pair_hash> grid;

    // 1. Popola la griglia
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        
        // Ignora punti fuori dal range massimo per velocizzare
        if (std::abs(pt.x) > max_range_ || std::abs(pt.y) > max_range_) continue;

        int grid_x = static_cast<int>(std::floor(pt.x / grid_res_));
        int grid_y = static_cast<int>(std::floor(pt.y / grid_res_));
        
        grid[{grid_x, grid_y}].point_indices.push_back(i);
    }

    int current_cluster_id = 1;

    // Definiamo i vicini da controllare (8-connected grid)
    const int dx[] = {-1, -1, -1,  0, 0,  1, 1, 1};
    const int dy[] = {-1,  0,  1, -1, 1, -1, 0, 1};

    // 2. Cerca componenti connesse (BFS)
    for (auto& [coord, cell] : grid) {
        // Se la cella è già stata visitata, salta
        if (cell.id != -1) continue;

        PointCloudPtr current_cluster(new PointCloud);
        std::queue<std::pair<int, int>> q;
        
        // Inizia un nuovo cluster
        cell.id = current_cluster_id;
        q.push(coord);

        while (!q.empty()) {
            auto curr = q.front();
            q.pop();

            // Aggiungi tutti i punti di questa cella al cluster corrente
            for (int idx : grid[curr].point_indices) {
                current_cluster->push_back(cloud->points[idx]);
            }

            // Controlla i vicini
            for (int i = 0; i < 8; ++i) {
                std::pair<int, int> neighbor = {curr.first + dx[i], curr.second + dy[i]};
                
                auto it = grid.find(neighbor);
                if (it != grid.end() && it->second.id == -1) {
                    it->second.id = current_cluster_id; // Marca come visitato
                    q.push(neighbor);
                }
            }
        }

        // Se il cluster rispetta le dimensioni, salvalo
        if (current_cluster->size() >= (size_t)min_size_ && current_cluster->size() <= (size_t)max_size_) {
            clusters.push_back(current_cluster);
        }
        
        current_cluster_id++;
    }
}

} // namespace fs_perception