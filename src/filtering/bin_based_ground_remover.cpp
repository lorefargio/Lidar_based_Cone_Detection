#include "filtering/bin_based_ground_remover.hpp"
#include <cmath>
#include <algorithm>

namespace fs_perception {

BinBasedGroundRemover::BinBasedGroundRemover() : config_(Config()) {
    grid_.resize(config_.segments * config_.bins);
}

BinBasedGroundRemover::BinBasedGroundRemover(const Config& config) : config_(config) {
    grid_.resize(config_.segments * config_.bins);
}

void BinBasedGroundRemover::removeGround(const PointCloudConstPtr& cloud_in, PointCloudPtr& cloud_obstacles, PointCloudPtr& cloud_ground) {
    const float BIN_SIZE = config_.max_range / config_.bins;

    // Reset grid structure for new frame
    Bin empty_bin; 
    empty_bin.min_z = std::numeric_limits<float>::max();
    empty_bin.has_points = false;
    std::fill(grid_.begin(), grid_.end(), empty_bin);

    // Phase 1: Local Parameterization (Polar Grid mapping)
    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        // Radius calculation
        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        if (r < 0.5f || r > config_.max_range) continue;

        // Angle calculation for sector assignment
        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / config_.segments)) % config_.segments;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= config_.bins) b_idx = config_.bins - 1;

        int idx = s_idx * config_.bins + b_idx; 

        // Update local minimum height (Lowest Point Representation - LPR)
        if (pt.z < grid_[idx].min_z) {
            grid_[idx].min_z = pt.z;
            grid_[idx].has_points = true;
        }
    }

    // Phase 2: Classification based on local height difference
    cloud_obstacles->points.reserve(cloud_in->points.size() / 2);
    cloud_ground->points.reserve(cloud_in->points.size() / 2);

    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        if (r < 0.5f || r > config_.max_range) continue;

        // Safety hard cutoff (all points below this level are ground)
        if (pt.z < config_.hard_ground_cutoff) {
            cloud_ground->push_back(pt);
            continue;
        }

        // Map point to its corresponding grid bin
        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / config_.segments)) % config_.segments;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= config_.bins) b_idx = config_.bins - 1;

        int idx = s_idx * config_.bins + b_idx;

        bool is_obstacle = false;

        // Check if the point is significantly higher than the bin's ground level
        if (grid_[idx].has_points) {
            float local_ground = grid_[idx].min_z;
            if (pt.z > (local_ground + config_.local_threshold)) {
                is_obstacle = true;
            }
        } else {
            // No local ground reference in this bin; assume obstacle for safety
            is_obstacle = true;
        }

        if (is_obstacle) {
            cloud_obstacles->push_back(pt);
        } else {
            cloud_ground->push_back(pt);
        }
    }
}

} // namespace fs_perception