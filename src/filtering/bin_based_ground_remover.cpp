#include "filtering/bin_based_ground_remover.hpp"
#include <cmath>
#include <algorithm>
#include <pcl/filters/voxel_grid.h>

namespace fs_perception {

BinBasedGroundRemover::BinBasedGroundRemover() : config_(Config()) {
    grid_.resize(config_.segments * config_.bins);
}

BinBasedGroundRemover::BinBasedGroundRemover(const Config& config) : config_(config) {
    grid_.resize(config_.segments * config_.bins);
}

void BinBasedGroundRemover::removeGround(const PointCloudConstPtr& cloud_in, PointCloudPtr& cloud_obstacles, PointCloudPtr& cloud_ground) {
    const float BIN_SIZE = config_.max_range / config_.bins;

    /**
     * @phase Grid Initialization
     * Re-initialization of the polar grid structure for the current frame to 
     * maintain temporal independence.
     */
    Bin empty_bin; 
    empty_bin.min_z = std::numeric_limits<float>::max();
    empty_bin.has_points = false;
    std::fill(grid_.begin(), grid_.end(), empty_bin);

    /**
     * @phase Local Minimum Identification (LPR)
     * Mapping of points to the polar grid to identify the Lowest Point Representation 
     * (LPR) within each discrete spatial bin.
     */
    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        if (r < 0.5f || r > config_.max_range) continue;

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / config_.segments)) % config_.segments;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= config_.bins) b_idx = config_.bins - 1;

        int idx = s_idx * config_.bins + b_idx; 

        if (pt.z < grid_[idx].min_z) {
            grid_[idx].min_z = pt.z;
            grid_[idx].has_points = true;
        }
    }

    /**
     * @phase Binary Segmentation
     * Classification of points into obstacle and ground categories based on 
     * local vertical displacement from the identified LPR.
     */
    cloud_obstacles->points.reserve(cloud_in->points.size() / 2);
    cloud_ground->points.reserve(cloud_in->points.size() / 2);

    for (const auto& pt : cloud_in->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        if (r < 0.5f || r > config_.max_range) continue;

        if (pt.z < config_.hard_ground_cutoff) {
            cloud_ground->push_back(pt);
            continue;
        }

        float angle = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        int s_idx = static_cast<int>(angle / (360.0f / config_.segments)) % config_.segments;
        int b_idx = static_cast<int>(r / BIN_SIZE);
        if (b_idx >= config_.bins) b_idx = config_.bins - 1;

        int idx = s_idx * config_.bins + b_idx;

        bool is_obstacle = false;

        if (grid_[idx].has_points) {
            float local_ground = grid_[idx].min_z;
            if (pt.z > (local_ground + config_.local_threshold)) {
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

    /**
     * @phase Post-processing Filtering
     * Optional voxelization of the obstacle cloud to reduce data density 
     * for downstream clustering algorithms.
     */
    if (voxel_size_ > 0.001f) {
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud(cloud_obstacles);
        voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        PointCloudPtr filtered(new PointCloud);
        voxel_grid.filter(*filtered);
        *cloud_obstacles = *filtered;
    }
}

} // namespace fs_perception