#include "filtering/patchworkpp_ground_remover.hpp"
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

namespace fs_perception {

PatchworkppGroundRemover::PatchworkppGroundRemover(const patchwork::Params& params) 
    : params_(params) {
    patchwork_ptr_ = std::make_unique<patchwork::PatchWorkpp>(params_);
}

void PatchworkppGroundRemover::removeGround(const PointCloudConstPtr& cloud_in, 
                                            PointCloudPtr& cloud_obstacles, 
                                            PointCloudPtr& cloud_ground) {
    if (cloud_in->empty() || !patchwork_ptr_) return;

    // 1. Snapshot and filtered Eigen mapping (Reusing persistent members)
    cloud_eigen_.resize(cloud_in->size(), 4); // Matrix with 4 columns: XYZI (Required for RNR)
    original_indices_.clear();
    original_indices_.reserve(cloud_in->size());
    
    int valid_count = 0;
    for (size_t i = 0; i < cloud_in->size(); ++i) {
        const auto& pt = cloud_in->points[i];
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            cloud_eigen_.row(valid_count) << (float)pt.x, (float)pt.y, (float)pt.z, (float)pt.intensity;
            original_indices_.push_back(i);
            valid_count++;
        }
    }

    if (valid_count == 0) return;

    // 2. Perform ground estimation (using optimized topRows block)
    patchwork_ptr_->estimateGround(cloud_eigen_.topRows(valid_count));

    // 3. Fast retrieval using indices (preserves ALL metadata like Intensity/Timestamp/Ring)
    Eigen::VectorXi ground_idx = patchwork_ptr_->getGroundIndices();
    Eigen::VectorXi nonground_idx = patchwork_ptr_->getNongroundIndices();

    auto populate_from_indices = [&](const Eigen::VectorXi& indices, PointCloudPtr& dst) {
        dst->points.clear();
        dst->points.reserve(indices.size());
        for (int i = 0; i < indices.size(); ++i) {
            int local_idx = indices[i];
            if (local_idx < (int)original_indices_.size()) {
                int global_idx = original_indices_[local_idx];
                dst->push_back(cloud_in->points[global_idx]);
            }
        }
        dst->width = dst->points.size();
        dst->height = 1;
        dst->is_dense = true;
        dst->header = cloud_in->header;
    };

    populate_from_indices(ground_idx, cloud_ground);
    populate_from_indices(nonground_idx, cloud_obstacles);

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

} // namespace fs_perception
