#pragma once

#include <memory>
#include <string>
#include <stdexcept>
#include "clustering/clusterer_interface.hpp"
#include "clustering/euclidean_clusterer.hpp"
#include "clustering/dbscan_clusterer.hpp"
#include "clustering/hdbscan_clusterer.hpp"
#include "clustering/voxel_connected_components.hpp"
#include "clustering/depth_clusterer.hpp"
#include "clustering/grid_clusterer.hpp"
#include "node/pipeline_config.hpp"

namespace lidar_perception {

/**
 * @class ClustererFactory
 * @brief Factory class to create clusterer instances based on configuration.
 */
class ClustererFactory {
public:
    static std::unique_ptr<ClustererInterface> create(const PipelineConfig& config) {
        int min_cluster = config.min_cluster_size;
        int max_cluster = config.max_cluster_size;
        float max_range = static_cast<float>(config.max_range);

        if (config.clustering_algorithm == "euclidean") {
            float tol = static_cast<float>(config.euclidean_tolerance);
            return std::make_unique<EuclideanClusterer>(tol, min_cluster, max_cluster);
        } else if (config.clustering_algorithm == "dbscan") {
            float eps = static_cast<float>(config.dbscan_eps);
            int min_pts = config.dbscan_min_pts;
            return std::make_unique<DBSCANClusterer>(eps, min_pts, min_cluster, max_cluster);
        } else if (config.clustering_algorithm == "hdbscan") {
            HDBSCANClusterer::Config cfg;
            cfg.min_pts = config.hdbscan_min_pts;
            cfg.alpha_scaling = static_cast<float>(config.hdbscan_alpha);
            cfg.min_cluster_size = min_cluster;
            return std::make_unique<HDBSCANClusterer>(cfg);
        } else if (config.clustering_algorithm == "voxel") {
            float v_size = static_cast<float>(config.voxel_grid_size);
            return std::make_unique<VoxelConnectedComponents>(v_size, min_cluster, max_cluster);
        } else if (config.clustering_algorithm == "depth") {
            DepthClusterer::Config cfg;
            cfg.theta_threshold_deg = static_cast<float>(config.depth_theta_thr);
            cfg.num_rings = config.depth_num_rings;
            cfg.min_cluster_size = min_cluster;
            cfg.max_cluster_size = max_cluster;
            return std::make_unique<DepthClusterer>(cfg);
        } else if (config.clustering_algorithm == "grid") {
            float res = static_cast<float>(config.grid_resolution);
            return std::make_unique<GridClusterer>(res, max_range, min_cluster, max_cluster);
        } else {
            throw std::invalid_argument("Unknown clustering algorithm: " + config.clustering_algorithm);
        }
    }
};

} // namespace lidar_perception
