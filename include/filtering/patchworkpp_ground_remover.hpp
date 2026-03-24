#pragma once

#include "filtering/ground_remover_interface.hpp"
#include <patchwork/patchworkpp.h>
#include <memory>

// patchworkpp_ground_remover.hpp
namespace fs_perception {

class PatchworkppGroundRemover : public GroundRemoverInterface {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Fondamentale!
    explicit PatchworkppGroundRemover(const patchwork::Params& params);
    
    void removeGround(const PointCloudConstPtr& cloud_in, 
                      PointCloudPtr& cloud_obstacles, 
                      PointCloudPtr& cloud_ground) override;

private:
    patchwork::Params params_;
    std::unique_ptr<patchwork::PatchWorkpp> patchwork_ptr_;

    // Membri persistenti per evitare allocazioni/distruzioni sullo stack
    Eigen::MatrixXf cloud_eigen_;
    Eigen::MatrixX3f ground_pts_;
    Eigen::MatrixX3f nonground_pts_;
};

} // namespace fs_perception

// namespace fs_perception
