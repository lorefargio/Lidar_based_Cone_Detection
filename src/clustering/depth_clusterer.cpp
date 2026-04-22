#include "clustering/depth_clusterer.hpp"
#include <cmath>
#include <queue>
#include <algorithm>

namespace fs_perception {

DepthClusterer::DepthClusterer() : DepthClusterer(Config()) {}

DepthClusterer::DepthClusterer(const Config& config) : config_(config) {
    theta_threshold_rad_ = config_.theta_threshold_deg * M_PI / 180.0f;
    // Hesai 40P Horizontal Res: 0.4 deg at 20Hz -> ~900 columns. 
    // Using 1024 for power-of-two alignment or 2048 for safety.
    hor_res_rad_ = (2.0f * M_PI) / static_cast<float>(config_.num_cols);
}

void DepthClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    // Phase 1: Robust Range Image Mapping
    const int num_rings = 40; // Fixed for Hesai 40P
    const int num_cols = config_.num_cols;
    std::vector<int> range_image(num_rings * num_cols, -1);
    std::vector<float> ranges(cloud->size());
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& pt = cloud->points[i];
        int row = pt.ring;
        if (row < 0 || row >= num_rings) continue;

        float azimuth = std::atan2(pt.y, pt.x);
        int col = static_cast<int>((azimuth + M_PI) / (2.0 * M_PI) * num_cols) % num_cols;

        // In case of voxelization collisions, keep the point closest to the sensor
        int idx = row * num_cols + col;
        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (range_image[idx] == -1 || r < ranges[range_image[idx]]) {
            range_image[idx] = static_cast<int>(i);
            ranges[i] = r;
        }
    }

    // Pre-calculate Vertical Resolutions for Hesai 40P
    auto get_v_res = [](int ring) {
        if (ring >= 6 && ring <= 30) return 0.33f * M_PI / 180.0f;
        if ((ring >= 5 && ring <= 6) || (ring >= 30 && ring <= 38)) return 1.0f * M_PI / 180.0f;
        if (ring >= 4 && ring <= 5) return 2.0f * M_PI / 180.0f;
        if (ring >= 2 && ring <= 4) return 3.0f * M_PI / 180.0f;
        if (ring >= 1 && ring <= 2) return 4.0f * M_PI / 180.0f;
        if (ring >= 38 && ring <= 39) return 5.0f * M_PI / 180.0f;
        return 6.0f * M_PI / 180.0f; // 39-40
    };

    // Phase 2: BFS with Neighborhood Window (Voxel-Proof)
    std::vector<int> labels(cloud->size(), -1);
    
    for (int r = 0; r < num_rings; ++r) {
        for (int c = 0; c < num_cols; ++c) {
            int pt_idx = range_image[r * num_cols + c];
            if (pt_idx == -1 || labels[pt_idx] != -1) continue;

            PointCloudPtr current_cluster(new PointCloud);
            std::queue<std::pair<int, int>> q;
            q.push({r, c});
            labels[pt_idx] = 1;

            while (!q.empty()) {
                auto [curr_r, curr_c] = q.front();
                q.pop();

                int curr_idx = range_image[curr_r * num_cols + curr_c];
                current_cluster->push_back(cloud->points[curr_idx]);

                // Window search: +/- 2 rings and +/- 2 columns
                // This bridges the gaps created by the 3.5cm voxel filter.
                for (int dr = -2; dr <= 2; ++dr) {
                    for (int dc = -2; dc <= 2; ++dc) {
                        if (dr == 0 && dc == 0) continue;

                        int next_r = curr_r + dr;
                        int next_c = (curr_c + dc + num_cols) % num_cols;

                        if (next_r < 0 || next_r >= num_rings) continue;

                        int next_idx = range_image[next_r * num_cols + next_c];
                        if (next_idx == -1 || labels[next_idx] != -1) continue;

                        // Angle-Beta calculation
                        float d1 = ranges[curr_idx];
                        float d2 = ranges[next_idx];

                        // Get angular resolution based on the jump size
                        float alpha;
                        if (dr != 0) {
                            // Vertical jump: sum of resolutions of jumped rings
                            alpha = std::abs(dr) * get_v_res(curr_r);
                        } else {
                            // Horizontal jump
                            alpha = std::abs(dc) * hor_res_rad_;
                        }

                        float d_min = std::min(d1, d2);
                        float d_max = std::max(d1, d2);
                        float beta = std::atan2(d_min * std::sin(alpha), d_max - d_min * std::cos(alpha));

                        // Accept if geometric continuity is maintained
                        if (beta > theta_threshold_rad_) {
                            labels[next_idx] = 1;
                            q.push({next_r, next_c});
                        }
                    }
                }
            }

            if (current_cluster->size() >= static_cast<size_t>(config_.min_cluster_size) &&
                current_cluster->size() <= static_cast<size_t>(config_.max_cluster_size)) {
                clusters.push_back(current_cluster);
            }
        }
    }
}

} // namespace fs_perception
