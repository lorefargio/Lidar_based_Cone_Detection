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
#include "clustering/string_clusterer.hpp"
#include "clustering/euclidean_clusterer.hpp"
#include "clustering/grid_clusterer.hpp"
#include "clustering/dbscan_clusterer.hpp"
#include "clustering/adaptive_dbscan_clusterer.hpp"
#include "clustering/voxel_connected_components.hpp"
#include "estimation/cone_estimator.hpp"

using namespace std::chrono_literals;

// Template instantiations for custom PCL point type
namespace pcl {
  template class PCLBase<fs_perception::PointT>;
  template class VoxelGrid<fs_perception::PointT>;
}

namespace fs_perception {

LidarPerceptionNode::LidarPerceptionNode() : Node("lidar_perception_node") {
    // --- 1. GENERAL PARAMETERS ---
    this->declare_parameter<std::string>("bag_path", "");
    this->declare_parameter<std::string>("clustering_algorithm", "grid");
    this->declare_parameter<std::string>("ground_remover_type", "slope_based");
    this->declare_parameter<std::string>("estimator_type", "rule_based");
    this->declare_parameter<std::string>("log_dir", "log_profiler/");
    this->declare_parameter<bool>("log_clusters", true);
    
    // Common geometric/filtering parameters
    this->declare_parameter<double>("sensor_z", -0.50);
    this->declare_parameter<double>("max_range", 25.0);
    this->declare_parameter<int>("min_cluster_size", 3);
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
        this->declare_parameter<double>("pw_min_range", 0.5);
        this->declare_parameter<double>("pw_uprightness_thr", 0.707);
        this->declare_parameter<bool>("pw_enable_RNR", true); 
        this->declare_parameter<bool>("pw_enable_TGR", true); // Terrain Gradient Robustness

        pw_params.sensor_height = std::abs(sensor_z);
        pw_params.max_range = max_range;
        pw_params.num_iter = this->get_parameter("pw_num_iter").as_int();
        pw_params.th_dist = this->get_parameter("pw_th_dist").as_double();
        pw_params.min_range = this->get_parameter("pw_min_range").as_double();
        pw_params.uprightness_thr = this->get_parameter("pw_uprightness_thr").as_double();
        pw_params.enable_RNR = this->get_parameter("pw_enable_RNR").as_bool();
        pw_params.enable_TGR = this->get_parameter("pw_enable_TGR").as_bool();

        ground_remover_ = std::make_unique<PatchworkppGroundRemover>(pw_params);
        RCLCPP_INFO(this->get_logger(), "Ground Removal: PATCHWORK++");
    }

    // Configure Modular Voxel Filter in Ground Remover
    this->declare_parameter<bool>("use_voxel_filter", false);
    this->declare_parameter<double>("voxel_size", 0.09); // Recommended for older hardware

    if (this->get_parameter("use_voxel_filter").as_bool() && ground_remover_) {
        double leaf_size = this->get_parameter("voxel_size").as_double();
        
        // Safety check for memory limits on older hardware
        if (leaf_size < 0.05) {
            RCLCPP_WARN(this->get_logger(), "Voxel size %.2f might cause memory issues on this PC. Using 0.09m is safer.", leaf_size);
        }
        
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
        this->declare_parameter<double>("hdbscan_eps_base", 0.15);
        this->declare_parameter<double>("hdbscan_alpha", 0.015);
        this->declare_parameter<int>("hdbscan_min_pts", 3);
        float eps_base = static_cast<float>(this->get_parameter("hdbscan_eps_base").as_double());
        float alpha = static_cast<float>(this->get_parameter("hdbscan_alpha").as_double());
        int min_pts = this->get_parameter("hdbscan_min_pts").as_int();
        clusterer_ = std::make_unique<AdaptiveDBSCANClusterer>(eps_base, alpha, min_pts, min_cluster, max_cluster);
        RCLCPP_INFO(this->get_logger(), "Clustering: ADAPTIVE-DBSCAN");
    } else if (cl_algo == "voxel") {
        this->declare_parameter<double>("voxel_grid_size", 0.15);
        float v_size = static_cast<float>(this->get_parameter("voxel_grid_size").as_double());
        clusterer_ = std::make_unique<VoxelConnectedComponents>(v_size, min_cluster, max_cluster);
        RCLCPP_INFO(this->get_logger(), "Clustering: VOXEL");
    } else if (cl_algo == "string") {
        StringClusterer::Config cfg;
        this->declare_parameter<double>("string_max_dist", 0.3);
        this->declare_parameter<double>("string_max_int_jump", 100.0);
        cfg.max_dist = static_cast<float>(this->get_parameter("string_max_dist").as_double());
        cfg.max_int_jump = static_cast<float>(this->get_parameter("string_max_int_jump").as_double());
        cfg.min_cluster_size = min_cluster;
        cfg.max_cluster_size = max_cluster;
        clusterer_ = std::make_unique<StringClusterer>(cfg);
        RCLCPP_INFO(this->get_logger(), "Clustering: STRING");
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
    
    ConeEstimator::Config est_cfg;
    this->declare_parameter<double>("rule_min_height", 0.10);
    this->declare_parameter<double>("rule_max_height", 0.50);
    this->declare_parameter<double>("rule_base_min_width", 0.10);
    this->declare_parameter<double>("rule_max_width", 0.36);
    this->declare_parameter<double>("rule_dynamic_width_decay", 0.005);
    this->declare_parameter<int>("rule_min_points_at_10m", 5);
    this->declare_parameter<double>("rule_min_intensity", 5.0);
    
    est_cfg.min_height = static_cast<float>(this->get_parameter("rule_min_height").as_double());
    est_cfg.max_height = static_cast<float>(this->get_parameter("rule_max_height").as_double());
    est_cfg.base_min_width = static_cast<float>(this->get_parameter("rule_base_min_width").as_double());
    est_cfg.max_width = static_cast<float>(this->get_parameter("rule_max_width").as_double());
    est_cfg.dynamic_width_decay = static_cast<float>(this->get_parameter("rule_dynamic_width_decay").as_double());
    est_cfg.min_points_at_10m = this->get_parameter("rule_min_points_at_10m").as_int();
    est_cfg.min_intensity = static_cast<float>(this->get_parameter("rule_min_intensity").as_double());
    est_cfg.max_linearity = static_cast<float>(this->get_parameter("pca_max_linearity").as_double());
    est_cfg.max_planarity = static_cast<float>(this->get_parameter("pca_max_planarity").as_double());
    est_cfg.min_scatter = static_cast<float>(this->get_parameter("pca_min_scatter").as_double());
    est_cfg.ground_z_level = sensor_z;

    estimator_ = std::make_unique<ConeEstimator>(est_cfg);
    RCLCPP_INFO(this->get_logger(), "Estimator: CONSOLIDATED CONE-ESTIMATOR");

    // --- 5. DESKEWING CONFIGURATION ---
    this->declare_parameter<bool>("use_deskewing", true);
    this->declare_parameter<std::string>("imu_topic", "/zed/zed_node/imu/data");
    this->declare_parameter<bool>("deskew_use_translation", true);
    this->declare_parameter<std::vector<double>>("static_imu_to_lidar_xyz", {0.0, 0.0, 0.0});

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
    }

    // --- 7. PUBLISHERS & SUBSCRIBERS ---
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_points", rclcpp::SensorDataQoS(), 
        std::bind(&LidarPerceptionNode::callback, this, std::placeholders::_1));

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
    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_debug", rclcpp::SensorDataQoS());
    pub_no_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/no_ground", rclcpp::SensorDataQoS());
}

LidarPerceptionNode::~LidarPerceptionNode() {
    if (profiler_) {
        profiler_->saveToJSON(json_file_path_);
        RCLCPP_INFO(this->get_logger(), "Profiling data saved to JSON before exit.");
    }
    if (cluster_logger_ && !csv_file_path_.empty()) {
        cluster_logger_->saveToCSV(csv_file_path_);
        RCLCPP_INFO(this->get_logger(), "Cluster analysis data saved to CSV before exit.");
    }
}

void LidarPerceptionNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!msg) return;

    frame_counter_++;
    if (profiler_) profiler_->startFrame();

    /**
     * @phase Data Conversion
     * Transformation of the incoming ROS 2 message into a PCL-compatible format.
     */
    if (profiler_) profiler_->startTimer("conversion");
    try {
        pcl::fromROSMsg(*msg, *raw_cloud_ptr_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "ROS-to-PCL conversion failure: %s", e.what());
        if (profiler_) profiler_->stopTimer("conversion");
        return;
    }
    if (profiler_) profiler_->stopTimer("conversion");
    
    if (raw_cloud_ptr_->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received null or empty point cloud.");
        return;
    }

    /**
     * @phase Pre-processing
     * Sequential execution of spatial truncation and early voxelization.
     * Truncation is performed first to bound the coordinate space, preventing 
     * potential overflow in the subsequent voxel grid filter.
     */
    if (profiler_) profiler_->startTimer("conversion"); 

    preProcess(raw_cloud_ptr_);

    if (raw_cloud_ptr_->size() > 500) {
        pcl::VoxelGrid<PointT> early_vg;
        early_vg.setInputCloud(raw_cloud_ptr_);
        early_vg.setLeafSize(0.035f, 0.035f, 0.035f); // 3.5cm isotropic voxel
        PointCloudPtr filtered(new PointCloud);
        early_vg.filter(*filtered);
        *raw_cloud_ptr_ = *filtered;
    }

    if (profiler_) profiler_->stopTimer("conversion");

    if (raw_cloud_ptr_->empty()) return;

    /**
     * @phase Motion Compensation
     * Deskewing of the point cloud based on high-frequency IMU telemetry.
     */
    if (this->get_parameter("use_deskewing").as_bool() && deskewer_) {
        if (profiler_) profiler_->startTimer("deskewing");
        if (!deskewer_->deskew(raw_cloud_ptr_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Deskewing operation postponed (awaiting IMU synchronization).");
        }
        if (profiler_) profiler_->stopTimer("deskewing");
    }

    /**
     * @phase Ground Removal
     * Segmentation of the point cloud into traversable terrain and obstacle subsets.
     */
    if (!ground_remover_) return;

    if (profiler_) profiler_->startTimer("ground_removal");
    
    obstacles_str_->clear();
    ground_str_->clear();

    ground_remover_->removeGround(raw_cloud_ptr_, obstacles_str_, ground_str_);

    if (profiler_) profiler_->stopTimer("ground_removal");
    
    if (frame_counter_ % this->get_parameter("debug_pub_freq").as_int() == 0) {
        sensor_msgs::msg::PointCloud2 no_ground_msg;
        pcl::toROSMsg(*obstacles_str_, no_ground_msg);
        no_ground_msg.header = msg->header;
        pub_no_ground_->publish(std::move(no_ground_msg));
    }
    
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
     * Proximity-based consolidation of over-segmented clusters using centroidal distance.
     */
    if (profiler_) profiler_->startTimer("merging");
    std::vector<PointCloudPtr> merged_clusters;
    
    std::vector<PointCloudPtr> valid_candidates;
    for (auto& c : clusters) {
        if (c->size() >= 3) {
            valid_candidates.push_back(c);
        }
    }

    std::vector<bool> merged_flag(valid_candidates.size(), false);
    
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids(valid_candidates.size());
    
    for(size_t i = 0; i < valid_candidates.size(); ++i) {
        pcl::compute3DCentroid(*valid_candidates[i], centroids[i]);
    }

    const float MERGE_DIST_SQ = 0.25f * 0.25f; 
    for (size_t i = 0; i < valid_candidates.size(); ++i) {
        if (merged_flag[i]) continue;
        PointCloudPtr super_cluster(new PointCloud(*valid_candidates[i]));

        for (size_t j = i + 1; j < valid_candidates.size(); ++j) {
            if (merged_flag[j]) continue;
            float dx = centroids[i][0] - centroids[j][0];
            float dy = centroids[i][1] - centroids[j][1];

            if (dx*dx + dy*dy < MERGE_DIST_SQ) {
                *super_cluster += *valid_candidates[j];
                merged_flag[j] = true; 
            }
        }
        merged_clusters.push_back(super_cluster);
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
        if (cone.confidence > 0.5f) {
            if (cluster_logger_) {
                cone.features.frame_id = frame_counter_;
                cone.features.cluster_id = static_cast<int>(i);
                cluster_logger_->addCluster(cone.features);
            }
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
    const float MIN_DIST_SQ = 0.45f * 0.45f; 
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
     * @phase Output Generation
     * Construction and publication of visualization markers and detection messages.
     * Uses std::move to efficiently transfer ownership of large message structures.
     */
    visualization_msgs::msg::MarkerArray markers;
    PointCloud cones_cloud_out; 
    PointCloud cone_points_out;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header = msg->header;
    delete_marker.ns = "cones";
    delete_marker.id = 0;
    markers.markers.push_back(delete_marker);

    visualization_msgs::msg::Marker delete_labels;
    delete_labels.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_labels.header = msg->header;
    delete_labels.ns = "distance_labels";
    delete_labels.id = 0;
    markers.markers.push_back(delete_labels);

    int id_counter = 1;
    for (const auto& cone : final_cones) {
        visualization_msgs::msg::Marker m;
        m.header = msg->header; 
        m.ns = "cones";
        m.id = id_counter;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.lifetime = rclcpp::Duration::from_seconds(0.1); 

        m.pose.position.x = cone.x;
        m.pose.position.y = cone.y;
        m.pose.position.z = cone.z + (cone.height / 2.0); 
        m.scale.x = 0.22; m.scale.y = 0.22; m.scale.z = cone.height;
        m.color.a = 0.8;
        
        if (cone.color == ConeColor::BLUE) { 
            m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f; 
        } else if (cone.color == ConeColor::YELLOW) { 
            m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; 
        } else { 
            m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f; 
        }
        markers.markers.push_back(m);

        visualization_msgs::msg::Marker t;
        t.header = msg->header;
        t.ns = "distance_labels";
        t.id = id_counter++;
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::msg::Marker::ADD;
        t.lifetime = rclcpp::Duration::from_seconds(0.1);
        t.pose.position.x = cone.x;
        t.pose.position.y = cone.y;
        t.pose.position.z = cone.z + cone.height + 0.25;
        t.scale.z = 0.15; 
        t.color.a = 1.0; t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0;
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << cone.range << "m";
        t.text = ss.str();
        markers.markers.push_back(t);

        PointT p_out;
        p_out.x = cone.x; p_out.y = cone.y; p_out.z = cone.z;
        p_out.intensity = cone.range; 
        cones_cloud_out.push_back(p_out);

        if (cone.cloud) {
            cone_points_out += *cone.cloud;
        }
    }

    pub_markers_->publish(std::move(markers));

    sensor_msgs::msg::PointCloud2 cones_msg;
    pcl::toROSMsg(cones_cloud_out, cones_msg);
    cones_msg.header = msg->header; 
    pub_cones_->publish(std::move(cones_msg));

    sensor_msgs::msg::PointCloud2 cone_points_msg;
    pcl::toROSMsg(cone_points_out, cone_points_msg);
    cone_points_msg.header = msg->header; 
    pub_cone_points_->publish(std::move(cone_points_msg));

    if (profiler_) {
        profiler_->endFrame(final_cones.size());

        if (frame_counter_ % 50 == 0) {
            auto total_ms = profiler_->getLastFrameTotalMs();
            RCLCPP_INFO(this->get_logger(), "Frame: %d | Latency: %.2f ms | Cones Detected: %zu", 
                frame_counter_, total_ms, final_cones.size());
            
            if (total_ms > 50.0) {
                RCLCPP_WARN(this->get_logger(), "Performance Violation: Frame latency (%.2f ms) exceeds 20Hz real-time budget.", total_ms);
            }
        }
    }
}

void LidarPerceptionNode::preProcess(PointCloudPtr cloud) {
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

} // namespace fs_perception

/**
 * @brief Main execution entry point.
 * 
 * Initializes the ROS 2 middleware and instantiates the perception controller.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<fs_perception::LidarPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
