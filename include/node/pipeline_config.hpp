#pragma once

#include <string>
#include <vector>

namespace lidar_perception {

/**
 * @struct PipelineConfig
 * @brief Structured configuration containing all hyperparameters for the LiDAR perception pipeline.
 */
struct PipelineConfig {
    // Pipeline selection
    std::string clustering_algorithm = "grid";
    std::string ground_remover_type = "patchworkpp";
    std::string estimator_type = "rule_based";
    std::string log_dir = "log_profiler/";
    std::string profile_name = "default";
    bool log_clusters = true;
    bool log_all_clusters = false;

    // Core physical/geometric thresholds
    double sensor_z = -0.52;
    double max_range = 25.0;
    int min_cluster_size = 2;
    int max_cluster_size = 300;
    int debug_pub_freq = 10;
    bool debug = true;

    // Voxel filter config
    bool use_voxel_filter = false;
    double voxel_size = 0.02;

    // Ground remover: bin_based
    double bin_local_threshold = 0.02;
    double bin_hard_cutoff = -0.47;
    int bin_segments = 500;
    int bin_bins = 500;

    // Ground remover: slope_based
    double slope_max_slope = 0.08;
    double slope_max_z_diff = 0.05;
    double slope_initial_threshold = 0.05;
    int slope_segments = 360;

    // Ground remover: patchworkpp
    int pw_num_iter = 3;
    double pw_th_dist = 0.02;
    double pw_th_seeds = 0.02;
    double pw_th_dist_v = 0.1;
    double pw_th_seeds_v = 0.02;
    double pw_min_range = 0.5;
    double pw_uprightness_thr = 0.707;
    bool pw_enable_RNR = true;
    bool pw_enable_TGR = true;

    // Clusterer: euclidean
    double euclidean_tolerance = 0.35;

    // Clusterer: dbscan
    double dbscan_eps = 0.30;
    int dbscan_min_pts = 3;

    // Clusterer: hdbscan
    int hdbscan_min_pts = 5;
    double hdbscan_alpha = 0.02;

    // Clusterer: voxel
    double voxel_grid_size = 0.02;

    // Clusterer: depth
    double depth_theta_thr = 10.0;
    int depth_num_rings = 32;

    // Clusterer: grid
    double grid_resolution = 0.12;

    // Estimator parameters
    double pca_max_linearity = 0.88;
    double pca_max_planarity = 0.8;
    double pca_min_scatter = 0.02;
    double pca_min_verticality = 0.65;
    
    double rule_min_height = 0.10;
    double rule_max_height = 0.80;
    double rule_base_min_width = 0.10;
    double rule_max_width = 0.50;
    double rule_dynamic_width_decay = 0.005;
    int rule_min_points_at_10m = 5;
    int rule_min_points_cap = 60;
    double rule_min_intensity = 5.0;

    // Post-processing & Aggregation
    double merge_dist = 0.25;
    double tracking_match_dist = 0.45;

    // Deskewing
    bool use_deskewing = true;
    std::string imu_topic = "/zed/zed_node/imu/data";
    std::string imu_frame = "zed_imu_link";
    std::vector<double> extrinsic_rotation = {0.999743, 0.0226629, 7.2829e-10, 8.06016e-10, -3.42052e-09, -1.0, -0.0226629, 0.999743, -3.43791e-09};
    std::vector<double> extrinsic_translation = {0.0543494, -0.0235914, -0.0488917};
    double roll_deg = -0.4;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    std::string world_up_axis = "y";
};

} // namespace lidar_perception
