#include "filtering/slope_based_ground_remover.hpp"
#include <algorithm>
#include <vector>
#include <pcl/filters/voxel_grid.h>

namespace lidar_perception {

SlopeBasedGroundRemover::SlopeBasedGroundRemover() : config_(Config()) {
    sectors_.resize(config_.segments);
}

SlopeBasedGroundRemover::SlopeBasedGroundRemover(const Config& config) : config_(config) {
    sectors_.resize(config_.segments);
}

void SlopeBasedGroundRemover::removeGround(const PointCloudConstPtr& cloud_in, PointCloudPtr& cloud_obstacles, PointCloudPtr& cloud_ground) {
    if (cloud_in->empty()) {
        return;
    }

    // Phase 1: Organize points into angular sectors (Reusing persistent sectors_)
    for (auto& s : sectors_) {
        s.clear();
        s.reserve(cloud_in->size() / config_.segments * 2); 
    }

    for (size_t i = 0; i < cloud_in->size(); ++i) {
        const auto& pt = cloud_in->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / config_.segments)) % config_.segments;
        sectors_[s_idx].push_back(i);
    }

    // Pre-reserve memory for performance
    cloud_obstacles->points.reserve(cloud_in->size());
    cloud_ground->points.reserve(cloud_in->size());

    // Phase 2: Perform slope-based analysis for each sector independently
    // This allows for parallel processing of different angular segments.
    for (int s = 0; s < config_.segments; ++s) {
        auto& sector_indices = sectors_[s];
        if (sector_indices.empty()) continue;

        // Radial Sorting: Crucial for the 1D slope traversal. Sorting by distance 
        // allows the algorithm to incrementally 'climb' the terrain.
        std::sort(sector_indices.begin(), sector_indices.end(), [&](int a, int b) {
            const auto& pt_a = cloud_in->points[a];
            const auto& pt_b = cloud_in->points[b];
            return (pt_a.x * pt_a.x + pt_a.y * pt_a.y) < (pt_b.x * pt_b.x + pt_b.y * pt_b.y);
        });

        float last_ground_r = 0.0f;
        float last_ground_z = config_.sensor_z; // Starting ground level from sensor height

        for (int idx : sector_indices) {
            const auto& pt = cloud_in->points[idx];
            float current_r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            
            // Hard cutoff for points directly underneath or too close to the sensor
            if (current_r < 0.5f) {
                 cloud_ground->push_back(pt);
                 continue;
            }

            // Slope logic: compare relative height (dz) and radial distance (dr)
            // Points are classified as ground if the slope (dz/dr) is low, 
            // indicating a smooth surface even if that surface is inclined.
            float dr = current_r - last_ground_r;
            float dz = std::abs(pt.z - last_ground_z);

            bool is_ground = false;

            if (dr > 0.05f) {
                // Sufficient distance between points; use slope-based classification
                float slope = dz / dr;
                if (slope < config_.max_slope && dz < config_.max_z_diff) {
                    is_ground = true;
                }
            } else {
                // Points too close radially (same ring); check absolute height diff only
                if (dz < config_.max_z_diff) {
                    is_ground = true;
                }
            }

            // Safety check for points near the car's body (hard cutoff)
            if (pt.z < (config_.sensor_z + config_.initial_ground_threshold) && current_r < 3.0f) {
                is_ground = true;
            }

            if (is_ground) {
                // If point is classified as ground, update the last known ground level
                cloud_ground->push_back(pt);
                last_ground_r = current_r;
                last_ground_z = pt.z;
            } else {
                // Point is classified as an obstacle
                cloud_obstacles->push_back(pt);
            }
        }
    }

    // Apply voxel filter to obstacles if enabled
    if (voxel_size_ > 0.001f) {
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud(cloud_obstacles);
        voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        PointCloudPtr filtered(new PointCloud);
        voxel_grid.filter(*filtered);
        *cloud_obstacles = *filtered;
    }
}

} // namespace lidar_perception