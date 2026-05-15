#include "node/perception_node.hpp"

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "filtering/bin_based_ground_remover.hpp"
#include "filtering/slope_based_ground_remover.hpp"
#include "filtering/patchworkpp_ground_remover.hpp"
#include "clustering/depth_clusterer.hpp"
#include "clustering/euclidean_clusterer.hpp"
#include "clustering/grid_clusterer.hpp"
#include "clustering/dbscan_clusterer.hpp"
#include "clustering/hdbscan_clusterer.hpp"
#include "clustering/voxel_connected_components.hpp"
#include "estimation/cone_estimator.hpp"

using namespace std::chrono_literals;

// Template instantiations for custom PCL point type
namespace pcl {
  template class PCLBase<lidar_perception::PointT>;
  template class VoxelGrid<lidar_perception::PointT>;
}

namespace lidar_perception {

PerceptionNode::PerceptionNode() : Node("lidar_perception_node") {
    // --- 1. GENERAL PARAMETERS ---
    this->declare_parameter<std::string>("bag_path", "");
    this->declare_parameter<std::string>("clustering_algorithm", "grid");
    this->declare_parameter<std::string>("ground_remover_type", "slope_based");
    this->declare_parameter<std::string>("estimator_type", "rule_based");
    this->declare_parameter<std::string>("log_dir", "log_profiler/");
    this->declare_parameter<bool>("log_clusters", true);
    this->declare_parameter<bool>("log_all_clusters", false);
    
    // Common geometric/filtering parameters
    this->declare_parameter<double>("sensor_z", -0.52);
    this->declare_parameter<double>("max_range", 25.0);
    this->declare_parameter<int>("min_cluster_size", 2);
    this->declare_parameter<int>("max_cluster_size", 300);
    
    std::string bag_path = this->get_parameter("bag_path").as_string();
    if (bag_path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "FATAL: 'bag_path' parameter is missing!");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    // --- 2. GROUND REMOVAL CONFIGURATION ---
    std::string gr_type = this->get_parameter("ground_remover_type").as_string();
    float sensor_z = static_cast<float>(this->get_parameter("sensor_z").as_double());
    float max_range = static_cast<float>(this->get_parameter("max_range").as_double());

    if (gr_type == "bin_based") {
        BinBasedGroundRemover::Config cfg;
        this->declare_parameter<double>("bin_local_threshold", 0.02);
        this->declare_parameter<double>("bin_hard_cutoff", -0.47);
        this->declare_parameter<int>("bin_segments", 500);
        this->declare_parameter<int>("bin_bins", 500);
        
        cfg.sensor_z = sensor_z;
        cfg.max_range = max_range;
        cfg.local_threshold = static_cast<float>(this->get_parameter("bin_local_threshold").as_double());
        cfg.hard_ground_cutoff = static_cast<float>(this->get_parameter("bin_hard_cutoff").as_double());
        cfg.segments = this->get_parameter("bin_segments").as_int();
        cfg.bins = this->get_parameter("bin_bins").as_int();
        
        ground_remover_ = std::make_unique<BinBasedGroundRemover>(cfg);
        RCLCPP_INFO(this->get_logger(), "Ground Removal: BIN-BASED");
    } else if (gr_type == "slope_based") {
        SlopeBasedGroundRemover::Config cfg;
        this->declare_parameter<double>("slope_max_slope", 0.08);
        this->declare_parameter<double>("slope_max_z_diff", 0.05);
        this->declare_parameter<double>("slope_initial_threshold", 0.05);
        this->declare_parameter<int>("slope_segments", 360);

        cfg.sensor_z = sensor_z;
        cfg.max_range = max_range;
        cfg.max_slope = static_cast<float>(this->get_parameter("slope_max_slope").as_double());
        cfg.max_z_diff = static_cast<float>(this->get_parameter("slope_max_z_diff").as_double());
        cfg.initial_ground_threshold = static_cast<float>(this->get_parameter("slope_initial_threshold").as_double());
        cfg.segments = this->get_parameter("slope_segments").as_int();

        ground_remover_ = std::make_unique<SlopeBasedGroundRemover>(cfg);
        RCLCPP_INFO(this->get_logger(), "Ground Removal: SLOPE-BASED");
    } else if (gr_type == "patchworkpp") {
        patchwork::Params pw_params;
        this->declare_parameter<int>("pw_num_iter", 3);
        this->declare_parameter<double>("pw_th_dist", 0.02);
        this->declare_parameter<double>("pw_th_seeds", 0.02);
        this->declare_parameter<double>("pw_th_dist_v", 0.1);
        this->declare_parameter<double>("pw_th_seeds_v", 0.02);
        this->declare_parameter<double>("pw_min_range", 0.5);
        this->declare_parameter<double>("pw_uprightness_thr", 0.707);
        this->declare_parameter<bool>("pw_enable_RNR", true); 
        this->declare_parameter<bool>("pw_enable_TGR", true); 

        pw_params.sensor_height = std::abs(sensor_z);
        pw_params.max_range = max_range;
        pw_params.num_iter = this->get_parameter("pw_num_iter").as_int();
        pw_params.th_dist = this->get_parameter("pw_th_dist").as_double();
        pw_params.th_seeds = this->get_parameter("pw_th_seeds").as_double();
        pw_params.th_dist_v = this->get_parameter("pw_th_dist_v").as_double();
        pw_params.th_seeds_v = this->get_parameter("pw_th_seeds_v").as_double();
        pw_params.min_range = this->get_parameter("pw_min_range").as_double();
        pw_params.uprightness_thr = this->get_parameter("pw_uprightness_thr").as_double();
        pw_params.enable_RNR = this->get_parameter("pw_enable_RNR").as_bool();
        pw_params.enable_TGR = this->get_parameter("pw_enable_TGR").as_bool();

        ground_remover_ = std::make_unique<PatchworkppGroundRemover>(pw_params);
        RCLCPP_INFO(this->get_logger(), "Ground Removal: PATCHWORK++");
    }

    // Configure Modular Voxel Filter in Ground Remover
    this->declare_parameter<bool>("use_voxel_filter", false);
    this->declare_parameter<double>("voxel_size", 0.02); // Recommended for older hardware

    if (this->get_parameter("use_voxel_filter").as_bool() && ground_remover_) {
        double leaf_size = this->get_parameter("voxel_size").as_double();
        
        ground_remover_->setVoxelFilter(static_cast<float>(leaf_size));
    }

    // --- 3. CLUSTERING CONFIGURATION ---
    std::string cl_algo = this->get_parameter("clustering_algorithm").as_string();
    int min_cluster = this->get_parameter("min_cluster_size").as_int();
    int max_cluster = this->get_parameter("max_cluster_size").as_int();

    if (cl_algo == "euclidean") {
        this->declare_parameter<double>("euclidean_tolerance", 0.35);
        float tol = static_cast<float>(this->get_parameter("euclidean_tolerance").as_double());
        clusterer_ = std::make_unique<EuclideanClusterer>(tol, min_cluster, max_cluster);
        RCLCPP_INFO(this->get_logger(), "Clustering: EUCLIDEAN");
    } else if (cl_algo == "dbscan") {
        this->declare_parameter<double>("dbscan_eps", 0.30);
        this->declare_parameter<int>("dbscan_min_pts", 3);
        float eps = static_cast<float>(this->get_parameter("dbscan_eps").as_double());
        int min_pts = this->get_parameter("dbscan_min_pts").as_int();
        clusterer_ = std::make_unique<DBSCANClusterer>(eps, min_pts, min_cluster, max_cluster);
        RCLCPP_INFO(this->get_logger(), "Clustering: DBSCAN");
    } else if (cl_algo == "hdbscan") {
        HDBSCANClusterer::Config cfg;
        this->declare_parameter<int>("hdbscan_min_pts", 5);
        this->declare_parameter<double>("hdbscan_alpha", 0.02);
        cfg.min_pts = this->get_parameter("hdbscan_min_pts").as_int();
        cfg.alpha_scaling = static_cast<float>(this->get_parameter("hdbscan_alpha").as_double());
        cfg.min_cluster_size = min_cluster;
        clusterer_ = std::make_unique<HDBSCANClusterer>(cfg);
        RCLCPP_INFO(this->get_logger(), "Clustering: TRUE HDBSCAN (Lidar-Aware)");
    } else if (cl_algo == "voxel") {
        this->declare_parameter<double>("voxel_grid_size", 0.15);
        float v_size = static_cast<float>(this->get_parameter("voxel_grid_size").as_double());
        clusterer_ = std::make_unique<VoxelConnectedComponents>(v_size, min_cluster, max_cluster);
        RCLCPP_INFO(this->get_logger(), "Clustering: VOXEL");
    } else if (cl_algo == "depth") {
        DepthClusterer::Config cfg;
        this->declare_parameter<double>("depth_theta_thr", 8.0);
        this->declare_parameter<int>("depth_num_rings", 32);
        cfg.theta_threshold_deg = static_cast<float>(this->get_parameter("depth_theta_thr").as_double());
        cfg.num_rings = this->get_parameter("depth_num_rings").as_int();
        cfg.min_cluster_size = min_cluster;
        cfg.max_cluster_size = max_cluster;
        clusterer_ = std::make_unique<DepthClusterer>(cfg);
        RCLCPP_INFO(this->get_logger(), "Clustering: DEPTH-IMAGE (BFS)");
    } else {
        this->declare_parameter<double>("grid_resolution", 0.12);
        float res = static_cast<float>(this->get_parameter("grid_resolution").as_double());
        clusterer_ = std::make_unique<GridClusterer>(res, max_range, min_cluster, max_cluster);
        RCLCPP_INFO(this->get_logger(), "Clustering: GRID");
    }

    // --- 4. ESTIMATION CONFIGURATION ---
    // PCA Params
    this->declare_parameter<double>("pca_max_linearity", 0.8);
    this->declare_parameter<double>("pca_max_planarity", 0.8);
    this->declare_parameter<double>("pca_min_scatter", 0.02);
    this->declare_parameter<double>("pca_min_verticality", 0.65);
    
    ConeEstimator::Config est_cfg;
    this->declare_parameter<double>("rule_min_height", 0.10);
    this->declare_parameter<double>("rule_max_height", 0.50);
    this->declare_parameter<double>("rule_base_min_width", 0.10);
    this->declare_parameter<double>("rule_max_width", 0.36);
    this->declare_parameter<double>("rule_dynamic_width_decay", 0.005);
    this->declare_parameter<int>("rule_min_points_at_10m", 5);
    this->declare_parameter<int>("rule_min_points_cap", 60);
    this->declare_parameter<double>("rule_min_intensity", 5.0);

    est_cfg.min_height = static_cast<float>(this->get_parameter("rule_min_height").as_double());
    est_cfg.max_height = static_cast<float>(this->get_parameter("rule_max_height").as_double());
    est_cfg.base_min_width = static_cast<float>(this->get_parameter("rule_base_min_width").as_double());
    est_cfg.max_width = static_cast<float>(this->get_parameter("rule_max_width").as_double());
    est_cfg.dynamic_width_decay = static_cast<float>(this->get_parameter("rule_dynamic_width_decay").as_double());
    est_cfg.min_points_at_10m = this->get_parameter("rule_min_points_at_10m").as_int();
    est_cfg.min_points_cap = this->get_parameter("rule_min_points_cap").as_int();
    est_cfg.min_intensity = static_cast<float>(this->get_parameter("rule_min_intensity").as_double());
    est_cfg.max_linearity = static_cast<float>(this->get_parameter("pca_max_linearity").as_double());
    est_cfg.max_planarity = static_cast<float>(this->get_parameter("pca_max_planarity").as_double());
    est_cfg.min_scatter = static_cast<float>(this->get_parameter("pca_min_scatter").as_double());
    est_cfg.min_verticality = static_cast<float>(this->get_parameter("pca_min_verticality").as_double());
    est_cfg.ground_z_level = sensor_z;

    estimator_ = std::make_unique<ConeEstimator>(est_cfg);
    RCLCPP_INFO(this->get_logger(), "Estimator: CONSOLIDATED CONE-ESTIMATOR");

    // --- 5. POST-PROCESSING & AGGREGATION ---
    this->declare_parameter<double>("merge_dist", 0.25);
    this->declare_parameter<double>("tracking_match_dist", 0.45);

    // --- 6. DESKEWING CONFIGURATION ---
    this->declare_parameter<bool>("use_deskewing", true);
    this->declare_parameter<std::string>("imu_topic", "/zed/zed_node/imu/data");
    this->declare_parameter<bool>("deskew_use_translation", true);
    this->declare_parameter<std::vector<double>>("static_imu_to_lidar_xyz", {-0.037, 0.0335, 0.053});

    if (this->get_parameter("use_deskewing").as_bool()) {
        Deskewer::Config d_cfg;
        d_cfg.use_translation = this->get_parameter("deskew_use_translation").as_bool();
        
        auto xyz = this->get_parameter("static_imu_to_lidar_xyz").as_double_array();
        if (xyz.size() == 3) {
            d_cfg.static_imu_to_lidar = Eigen::Vector3f(xyz[0], xyz[1], xyz[2]);
        }
        
        deskewer_ = std::make_unique<Deskewer>(d_cfg);
        RCLCPP_INFO(this->get_logger(), "Deskewing: ENABLED");
    }

    // --- 6. POST-PROCESSING & BENCHMARKING ---
    std::string log_dir = this->get_parameter("log_dir").as_string();
    if (!log_dir.empty() && log_dir.back() != '/') log_dir += '/';
    std::filesystem::create_directories(log_dir); 
    
    this->declare_parameter<int>("debug_pub_freq", 10); // Publish debug clouds every 10 frames
    
    // Combine clustering and ground removal algorithm names for profiling
    std::string profile_name = cl_algo + "_" + gr_type;
    profiler_ = std::make_unique<PerformanceProfiler>(profile_name);
    json_file_path_ = log_dir + "profiler_" + profile_name + ".json";

    if (this->get_parameter("log_clusters").as_bool()) {
        cluster_logger_ = std::make_unique<ClusterLogger>(profile_name);
        csv_file_path_ = log_dir + "clusters_" + profile_name + ".csv";
        config_json_file_path_ = log_dir + "config_" + profile_name + ".json";
    }

    // --- 7. PUBLISHERS & SUBSCRIBERS ---
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_points", rclcpp::SensorDataQoS(), 
        std::bind(&PerceptionNode::callback, this, std::placeholders::_1));

    if (this->get_parameter("use_deskewing").as_bool()) {
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), 100, 
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                if (deskewer_) deskewer_->addImuMessage(msg);
            });
    }

    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/cones_vis", rclcpp::SensorDataQoS());
    pub_cones_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/cones", rclcpp::SensorDataQoS());
    pub_cone_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/cone_points", rclcpp::SensorDataQoS());
    pub_lidar_viz_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar_viz", rclcpp::SensorDataQoS());

    // Start background worker for visualization
    viz_thread_ = std::thread(&PerceptionNode::visualizationWorker, this);
}

PerceptionNode::~PerceptionNode() {
    {
        std::lock_guard<std::mutex> lock(viz_mutex_);
        stop_viz_thread_ = true;
    }
    viz_cv_.notify_all();
    if (viz_thread_.joinable()) viz_thread_.join();

    if (profiler_) {
        profiler_->saveToJSON(json_file_path_);
        RCLCPP_INFO(this->get_logger(), "Profiling data saved to JSON before exit.");
    }
    if (cluster_logger_ && !csv_file_path_.empty()) {
        cluster_logger_->saveToCSV(csv_file_path_);
        saveConfig(config_json_file_path_);
        RCLCPP_INFO(this->get_logger(), "Cluster analysis data and config saved before exit.");
    }
}

void PerceptionNode::saveConfig(const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) return;

    auto params = this->get_parameters(this->list_parameters({}, 0).names);
    
    out << "{\n";
    for (size_t i = 0; i < params.size(); ++i) {
        out << "  \"" << params[i].get_name() << "\": ";
        if (params[i].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            out << "\"" << params[i].as_string() << "\"";
        } else if (params[i].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            out << params[i].as_double();
        } else if (params[i].get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            out << params[i].as_int();
        } else if (params[i].get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            out << (params[i].as_bool() ? "true" : "false");
        } else if (params[i].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            out << "[";
            auto arr = params[i].as_double_array();
            for (size_t j = 0; j < arr.size(); ++j) {
                out << arr[j] << (j < arr.size() - 1 ? ", " : "");
            }
            out << "]";
        } else {
            out << "null";
        }
        out << (i < params.size() - 1 ? ",\n" : "\n");
    }
    out << "}\n";
    out.close();
}

void PerceptionNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!msg) return;

    frame_counter_++;
    if (profiler_) profiler_->startFrame();

    /**
     * @phase Data Conversion
     * Transformation of the incoming ROS 2 message into a PCL-compatible format.
     */
    if (profiler_) profiler_->startTimer("conversion");
    
    // Always use the persistent input buffer for ROS conversion
    try {
        pcl::fromROSMsg(*msg, *raw_cloud_ptr_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "ROS-to-PCL conversion failure: %s", e.what());
        if (profiler_) profiler_->stopTimer("conversion");
        return;
    }
    
    if (raw_cloud_ptr_->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received null or empty point cloud.");
        if (profiler_) profiler_->stopTimer("conversion");
        return;
    }

    preProcess(raw_cloud_ptr_);

    // --- Intensity Distance Normalization ---
    const float REF_DIST_SQ_INV = 1.0f / (10.0f * 10.0f);
    for (auto& pt : raw_cloud_ptr_->points) {
        float r2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
        if (r2 > 1.0f) {
            float factor = r2 * REF_DIST_SQ_INV;
            pt.intensity = std::min(255.0f, pt.intensity * factor); 
        }
    }

    // Pointer to the cloud we will actually process (start with raw)
    PointCloudPtr processing_cloud = raw_cloud_ptr_;

    if (raw_cloud_ptr_->size() > 500) {
        pcl::VoxelGrid<PointT> early_vg;
        early_vg.setInputCloud(raw_cloud_ptr_);
        double v_size = this->get_parameter("voxel_size").as_double();
        early_vg.setLeafSize(v_size, v_size, v_size);
        early_vg.filter(*filtered_cloud_ptr_);
        
        // Switch to the filtered cloud for the rest of the pipeline
        processing_cloud = filtered_cloud_ptr_; 

        // Publish a lightweight version for Foxglove/RViz at FULL frequency
        auto viz_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*filtered_cloud_ptr_, *viz_msg);
        viz_msg->header = msg->header;
        pub_lidar_viz_->publish(std::move(viz_msg));
        }

    if (profiler_) profiler_->stopTimer("conversion");

    if (processing_cloud->empty()) return;

    /**
     * @phase Motion Compensation
     */
    if (this->get_parameter("use_deskewing").as_bool() && deskewer_) {
        if (profiler_) profiler_->startTimer("deskewing");
        if (!deskewer_->deskew(processing_cloud)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Deskewing operation postponed (awaiting IMU synchronization).");
        }
        if (profiler_) profiler_->stopTimer("deskewing");
    }

    /**
     * @phase Ground Removal
     */
    if (!ground_remover_) return;

    if (profiler_) profiler_->startTimer("ground_removal");
    
    obstacles_str_->clear();
    ground_str_->clear();

    ground_remover_->removeGround(processing_cloud, obstacles_str_, ground_str_);

    if (profiler_) profiler_->stopTimer("ground_removal");
    
    /**
     * @phase Object Clustering
     * Identification of discrete point clusters using the configured spatial partitioning strategy.
     */
    if (!clusterer_) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Clustering interface not initialized.");
        return;
    }

    if (profiler_) profiler_->startTimer("clustering");
    std::vector<PointCloudPtr> clusters;
    clusterer_->cluster(obstacles_str_, clusters);
    if (profiler_) profiler_->stopTimer("clustering");

    /**
     * @phase Cluster Merging
     */
    if (profiler_) profiler_->startTimer("merging");
    std::vector<PointCloudPtr> merged_clusters;
    
    std::vector<PointCloudPtr> valid_candidates;
    valid_candidates.reserve(clusters.size());
    int min_cluster = this->get_parameter("min_cluster_size").as_int();
    for (auto& c : clusters) {
        if (c->size() >= static_cast<size_t>(min_cluster)) valid_candidates.push_back(c);
    }

    std::vector<bool> merged_flag(valid_candidates.size(), false);
    std::vector<Eigen::Vector3f> centroids(valid_candidates.size());
    
    for(size_t i = 0; i < valid_candidates.size(); ++i) {
        Eigen::Vector4f c4;
        pcl::compute3DCentroid(*valid_candidates[i], c4);
        centroids[i] = c4.head<3>();
    }

    const float merge_dist = static_cast<float>(this->get_parameter("merge_dist").as_double());
    const float MERGE_DIST_SQ = merge_dist * merge_dist; 
    for (size_t i = 0; i < valid_candidates.size(); ++i) {
        if (merged_flag[i]) continue;
        
        PointCloudPtr current_cloud = valid_candidates[i];
        bool has_merged = false;

        for (size_t j = i + 1; j < valid_candidates.size(); ++j) {
            if (merged_flag[j]) continue;
            float dx = centroids[i][0] - centroids[j][0];
            float dy = centroids[i][1] - centroids[j][1];

            if (dx*dx + dy*dy < MERGE_DIST_SQ) {
                if (!has_merged) {
                    // Only copy if we are actually merging
                    current_cloud.reset(new PointCloud(*valid_candidates[i]));
                    has_merged = true;
                }
                *current_cloud += *valid_candidates[j];
                merged_flag[j] = true; 
            }
        }
        merged_clusters.push_back(current_cloud);
    }
    if (profiler_) profiler_->stopTimer("merging");

    /**
     * @phase Feature Extraction and Estimation
     * Statistical classification of clusters using PCA-derived features and geometric constraints.
     */
    if (!estimator_) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Estimation engine not initialized.");
        return;
    }

    if (profiler_) profiler_->startTimer("estimation");
    std::vector<Cone> candidate_cones;

    for (size_t i = 0; i < merged_clusters.size(); ++i) {
        auto cone = estimator_->estimate(merged_clusters[i]);
        
        // Log cluster features if logging is enabled
        if (cluster_logger_) {
            if (this->get_parameter("log_all_clusters").as_bool()) {
                // Log everything to analyze rejected clusters
                cone.features.frame_id = frame_counter_;
                cone.features.cluster_id = static_cast<int>(i);
                cluster_logger_->addCluster(cone.features);
            } else if (cone.confidence > 0.5f) {
                // Log only successful detections (default)
                cone.features.frame_id = frame_counter_;
                cone.features.cluster_id = static_cast<int>(i);
                cluster_logger_->addCluster(cone.features);
            }
        }

        if (cone.confidence > 0.5f) {
            candidate_cones.push_back(cone);
        }
    }
    if (profiler_) profiler_->stopTimer("estimation");
    
    /**
     * @phase Weighted Spatial Aggregation
     * Non-maximum suppression with spatial averaging to eliminate redundant 
     * detections and smooth the reported position.
     */
    if (profiler_) profiler_->startTimer("duplicate");
    const float tracking_match_dist = static_cast<float>(this->get_parameter("tracking_match_dist").as_double());
    const float MIN_DIST_SQ = tracking_match_dist * tracking_match_dist; 
    std::vector<Cone> final_cones;
    std::vector<bool> candidate_merged(candidate_cones.size(), false);

    for (size_t i = 0; i < candidate_cones.size(); ++i) {
        if (candidate_merged[i]) continue;
        
        Cone aggregated = candidate_cones[i];
        float sum_x = aggregated.x;
        float sum_y = aggregated.y;
        float sum_z = aggregated.z;
        int count = 1;

        for (size_t j = i + 1; j < candidate_cones.size(); ++j) {
            if (candidate_merged[j]) continue;
            
            float dist_sq = (candidate_cones[i].x - candidate_cones[j].x)*(candidate_cones[i].x - candidate_cones[j].x) +
                            (candidate_cones[i].y - candidate_cones[j].y)*(candidate_cones[i].y - candidate_cones[j].y);
            
            if (dist_sq < MIN_DIST_SQ) {
                sum_x += candidate_cones[j].x;
                sum_y += candidate_cones[j].y;
                sum_z += candidate_cones[j].z;
                count++;
                candidate_merged[j] = true;
            }
        }
        
        if (count > 1) {
            aggregated.x = sum_x / count;
            aggregated.y = sum_y / count;
            aggregated.z = sum_z / count;
            aggregated.range = std::sqrt(aggregated.x*aggregated.x + aggregated.y*aggregated.y);
            aggregated.bearing = std::atan2(aggregated.y, aggregated.x);
        }
        final_cones.push_back(aggregated);
    }
    if (profiler_) profiler_->stopTimer("duplicate");

    /**
     * @phase Asynchronous Handover
     * Transfer processed data to the visualization worker thread and return control 
     * to the main callback immediately.
     */
    {
        std::lock_guard<std::mutex> lock(viz_mutex_);
        next_viz_data_ = std::make_unique<VizData>();
        next_viz_data_->header = msg->header;
        
        // Data snapshots to avoid race conditions
        next_viz_data_->cloud_viz.reset(new PointCloud(*processing_cloud));
        next_viz_data_->cones_cloud.reset(new PointCloud);
        next_viz_data_->cones_cloud->reserve(final_cones.size());
        next_viz_data_->cone_points.reset(new PointCloud);
        next_viz_data_->cone_points->reserve(final_cones.size() * 30);
        
        // Marker generation
        auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
        markers->markers.reserve(final_cones.size() * 2 + 2);

        visualization_msgs::msg::Marker delete_template;
        delete_template.header = msg->header;
        delete_template.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_template.id = 0;
        delete_template.ns = "cones";
        markers->markers.push_back(delete_template);
        delete_template.ns = "distance_labels";
        markers->markers.push_back(delete_template);

        visualization_msgs::msg::Marker cone_template;
        cone_template.header = msg->header;
        cone_template.ns = "cones";
        cone_template.type = visualization_msgs::msg::Marker::CYLINDER;
        cone_template.action = visualization_msgs::msg::Marker::ADD;
        cone_template.lifetime = rclcpp::Duration::from_seconds(0.2);
        cone_template.scale.x = 0.22; cone_template.scale.y = 0.22;
        cone_template.color.a = 1.0; 

        visualization_msgs::msg::Marker text_template;
        text_template.header = msg->header;
        text_template.ns = "distance_labels";
        text_template.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_template.action = visualization_msgs::msg::Marker::ADD;
        text_template.lifetime = rclcpp::Duration::from_seconds(0.2);
        text_template.scale.z = 0.15;
        text_template.color.a = 1.0; text_template.color.r = 1.0; text_template.color.g = 1.0; text_template.color.b = 1.0;

        int id_counter = 1;
        for (const auto& cone : final_cones) {
            // Cone Marker
            cone_template.id = id_counter;
            cone_template.pose.position.x = cone.x;
            cone_template.pose.position.y = cone.y;
            cone_template.pose.position.z = cone.z + (cone.height / 2.0); 
            cone_template.scale.z = std::max(0.1f, cone.height);
            
            if (cone.color == ConeColor::YELLOW) {
                cone_template.color.r = 1.0f; cone_template.color.g = 1.0f; cone_template.color.b = 0.0f;
            } else if (cone.color == ConeColor::BLUE) {
                cone_template.color.r = 0.0f; cone_template.color.g = 0.0f; cone_template.color.b = 1.0f;
            } else {
                cone_template.color.r = 0.5f; cone_template.color.g = 0.5f; cone_template.color.b = 0.5f;
            }
            markers->markers.push_back(cone_template);

            // Text Label
            text_template.id = id_counter++;
            text_template.pose.position.x = cone.x;
            text_template.pose.position.y = cone.y;
            text_template.pose.position.z = cone.z + cone.height + 0.20;
            char dist_str[16];
            snprintf(dist_str, sizeof(dist_str), "%.1fm", cone.range);
            text_template.text = dist_str;
            markers->markers.push_back(text_template);

            // PointCloud Data
            PointT p_out;
            p_out.x = cone.x; p_out.y = cone.y; p_out.z = cone.z;
            p_out.intensity = cone.range; // Carry radial range as per fusion_integration.md
            next_viz_data_->cones_cloud->push_back(p_out);
            
            if (cone.cloud) {
                *(next_viz_data_->cone_points) += *cone.cloud;
            }
        }
        next_viz_data_->markers = std::move(markers);
    }
    viz_cv_.notify_one();

    if (profiler_) {
        profiler_->endFrame(final_cones.size());

        if (frame_counter_ % 1 == 0) {
            auto total_ms = profiler_->getLastFrameTotalMs();
            RCLCPP_INFO(this->get_logger(), "Frame: %d | Latency: %.2f ms | Cones Detected: %zu", 
                frame_counter_, total_ms, final_cones.size());
            if (total_ms > 20.0) {
                RCLCPP_WARN(this->get_logger(), "Performance Violation: Frame latency (%.2f ms) exceeds 20ms real-time budget.", total_ms);
            }
        }
    }
}

void PerceptionNode::visualizationWorker() {
    while (rclcpp::ok()) {
        std::unique_ptr<VizData> data;
        {
            std::unique_lock<std::mutex> lock(viz_mutex_);
            viz_cv_.wait(lock, [this] { return next_viz_data_ || stop_viz_thread_; });
            
            if (stop_viz_thread_) break;
            
            data = std::move(next_viz_data_);
        }

        if (!data) continue;

        // 1. Markers
        if (data->markers) {
            pub_markers_->publish(std::move(data->markers));
        }

        // 2. Lidar Viz
        if (data->cloud_viz) {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*(data->cloud_viz), *msg);
            msg->header = data->header;
            pub_lidar_viz_->publish(std::move(msg));
        }

        // 3. Cones
        if (data->cones_cloud) {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*(data->cones_cloud), *msg);
            msg->header = data->header;
            pub_cones_->publish(std::move(msg));
        }

        // 4. Cone Points
        if (data->cone_points) {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*(data->cone_points), *msg);
            msg->header = data->header;
            pub_cone_points_->publish(std::move(msg));
        }
    }
}

void PerceptionNode::preProcess(const PointCloudPtr& cloud) {
    if (cloud->empty()) return;

    /**
     * @details Implements a rapid spatial filter to prune points outside the 
     * operational region of interest. Distances are evaluated as squared values 
     * to circumvent costly square-root operations.
     */
    const float max_range = static_cast<float>(this->get_parameter("max_range").as_double());
    const float max_r2 = max_range * max_range;
    const float min_r2 = 0.25f * 0.25f; // Internal occlusion / sensor blind spot

    auto it = std::remove_if(cloud->points.begin(), cloud->points.end(), [&](const PointT& pt) {
        float r2 = pt.x * pt.x + pt.y * pt.y;
        return r2 > max_r2 || r2 < min_r2 || !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z);
    });

    if (it != cloud->points.end()) {
        cloud->points.erase(it, cloud->points.end());
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
}

} // namespace lidar_perception

/**
 * @brief Main execution entry point.
 * 
 * Initializes the ROS 2 middleware and instantiates the perception controller.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lidar_perception::PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
