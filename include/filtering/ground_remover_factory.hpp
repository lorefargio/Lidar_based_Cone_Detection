#pragma once

#include <memory>
#include <string>
#include <stdexcept>
#include "filtering/ground_remover_interface.hpp"
#include "filtering/bin_based_ground_remover.hpp"
#include "filtering/slope_based_ground_remover.hpp"
#include "filtering/patchworkpp_ground_remover.hpp"
#include "node/pipeline_config.hpp"

namespace lidar_perception {

/**
 * @class GroundRemoverFactory
 * @brief Factory class to create ground remover instances based on the configuration.
 */
class GroundRemoverFactory {
public:
    static std::unique_ptr<GroundRemoverInterface> create(const PipelineConfig& config) {
        std::unique_ptr<GroundRemoverInterface> remover;
        
        float sensor_z = static_cast<float>(config.sensor_z);
        float max_range = static_cast<float>(config.max_range);

        if (config.ground_remover_type == "bin_based") {
            BinBasedGroundRemover::Config cfg;
            cfg.sensor_z = sensor_z;
            cfg.max_range = max_range;
            cfg.local_threshold = static_cast<float>(config.bin_local_threshold);
            cfg.hard_ground_cutoff = static_cast<float>(config.bin_hard_cutoff);
            cfg.segments = config.bin_segments;
            cfg.bins = config.bin_bins;
            
            remover = std::make_unique<BinBasedGroundRemover>(cfg);
        } else if (config.ground_remover_type == "slope_based") {
            SlopeBasedGroundRemover::Config cfg;
            cfg.sensor_z = sensor_z;
            cfg.max_range = max_range;
            cfg.max_slope = static_cast<float>(config.slope_max_slope);
            cfg.max_z_diff = static_cast<float>(config.slope_max_z_diff);
            cfg.initial_ground_threshold = static_cast<float>(config.slope_initial_threshold);
            cfg.segments = config.slope_segments;

            remover = std::make_unique<SlopeBasedGroundRemover>(cfg);
        } else if (config.ground_remover_type == "patchworkpp") {
            patchwork::Params pw_params;
            pw_params.sensor_height = std::abs(sensor_z);
            pw_params.max_range = max_range;
            pw_params.num_iter = config.pw_num_iter;
            pw_params.th_dist = config.pw_th_dist;
            pw_params.th_seeds = config.pw_th_seeds;
            pw_params.th_dist_v = config.pw_th_dist_v;
            pw_params.th_seeds_v = config.pw_th_seeds_v;
            pw_params.min_range = config.pw_min_range;
            pw_params.uprightness_thr = config.pw_uprightness_thr;
            pw_params.enable_RNR = config.pw_enable_RNR;
            pw_params.enable_TGR = config.pw_enable_TGR;

            remover = std::make_unique<PatchworkppGroundRemover>(pw_params);
        } else {
            throw std::invalid_argument("Unknown ground remover type: " + config.ground_remover_type);
        }

        if (config.use_voxel_filter && remover) {
            remover->setVoxelFilter(static_cast<float>(config.voxel_size));
        }

        return remover;
    }
};

} // namespace lidar_perception
