#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <patchwork/patchworkpp.h>
#include <memory>

// patchworkpp_ground_remover.hpp
namespace fs_perception {

/**
 * @class PatchworkppGroundRemover
 * @brief High-performance ground removal using the Patchwork++ algorithm.
 * 
 * Patchwork++ is a state-of-the-art ground segmentation algorithm that handles 
 * non-flat terrain and complex environments using Concentric Zone Model (CZM) 
 * and Region-wise Ground Fitting (RGF). This class provides a PCL-compatible 
 * wrapper around the core implementation.
 */
class PatchworkppGroundRemover : public GroundRemoverInterface {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit PatchworkppGroundRemover(const patchwork::Params& params);
    
    void removeGround(const PointCloudConstPtr& cloud_in, 
                      PointCloudPtr& cloud_obstacles, 
                      PointCloudPtr& cloud_ground) override;

private:
    patchwork::Params params_;
    std::unique_ptr<patchwork::PatchWorkpp> patchwork_ptr_;

    // Persistent members from working snippet
    Eigen::MatrixXf cloud_eigen_;
    std::vector<int> original_indices_;
};

} // namespace fs_perception

// namespace fs_perception
