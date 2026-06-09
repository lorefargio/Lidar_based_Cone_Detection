#include "node/perception_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <iomanip>

namespace lidar_perception {

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions& options) 
    : Node("perception_node", options) {
    
    // 1. Declare and load parameters
    initializeParameters();
    loadParametersToConfig();

    debug_mode_ = config_.debug;

    // 2. Set up logging and diagnostics
    std::string profile_name = config_.profile_name;
    std::string log_dir = config_.log_dir;
    if (!log_dir.empty() && log_dir.back() != '/') {
        log_dir += "/";
    }

    profiler_ = std::make_unique<PerformanceProfiler>(config_.clustering_algorithm + "_" + config_.ground_remover_type);
    json_file_path_ = log_dir + "profiler_" + profile_name + ".json";

    if (config_.log_clusters) {
        cluster_logger_ = std::make_unique<ClusterLogger>(profile_name);
        csv_file_path_ = log_dir + "clusters_" + profile_name + ".csv";
        config_json_file_path_ = log_dir + "config_" + profile_name + ".json";
    }

    // 3. Create core pipeline facade and visualizer bridge
    pipeline_ = std::make_unique<PerceptionPipeline>();
    pipeline_->initialize(config_, this->get_clock());

    viz_bridge_ = std::make_unique<VisualizationBridge>(this);
    clock_aligner_ = std::make_unique<fs_fusion::ClockAligner>();

    // 4. Set up communication interfaces
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar_points", rclcpp::SensorDataQoS(), 
        std::bind(&PerceptionNode::callback, this, std::placeholders::_1));

    if (config_.use_deskewing) {
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            config_.imu_topic, 100, 
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                auto interpolator = pipeline_->getImuInterpolator();
                if (interpolator) {
                    fs_fusion::ImuData data;
                    uint64_t ts_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
                    if (clock_aligner_) {
                        clock_aligner_->updateCameraTime(ts_ns, this->get_logger());
                    }
                    data.ts_ns = ts_ns;
                    data.angular_vel = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                    data.linear_accel = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                    interpolator->addImuData(data);
                }
            });
    }

    RCLCPP_INFO(this->get_logger(), "Perception wrapper node initialized successfully.");
}

PerceptionNode::~PerceptionNode() {
    if (viz_bridge_) viz_bridge_->stop();

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

void PerceptionNode::initializeParameters() {
    this->declare_parameter<std::string>("clustering_algorithm", "grid");
    this->declare_parameter<std::string>("ground_remover_type", "patchworkpp");
    this->declare_parameter<std::string>("estimator_type", "rule_based");
    this->declare_parameter<std::string>("log_dir", "log_profiler/");
    this->declare_parameter<std::string>("profile_name", "default");
    this->declare_parameter<bool>("log_clusters", true);
    this->declare_parameter<bool>("log_all_clusters", false);
    
    this->declare_parameter<double>("sensor_z", -0.52);
    this->declare_parameter<double>("max_range", 25.0);
    this->declare_parameter<int>("min_cluster_size", 2);
    this->declare_parameter<int>("max_cluster_size", 300);
    this->declare_parameter<int>("debug_pub_freq", 10);
    this->declare_parameter<bool>("debug", true);

    // Voxel
    this->declare_parameter<bool>("use_voxel_filter", false);
    this->declare_parameter<double>("voxel_size", 0.02);

    // Bin based
    this->declare_parameter<double>("bin_local_threshold", 0.02);
    this->declare_parameter<double>("bin_hard_cutoff", -0.47);
    this->declare_parameter<int>("bin_segments", 500);
    this->declare_parameter<int>("bin_bins", 500);

    // Slope based
    this->declare_parameter<double>("slope_max_slope", 0.08);
    this->declare_parameter<double>("slope_max_z_diff", 0.05);
    this->declare_parameter<double>("slope_initial_threshold", 0.05);
    this->declare_parameter<int>("slope_segments", 360);

    // Patchwork++
    this->declare_parameter<int>("pw_num_iter", 3);
    this->declare_parameter<double>("pw_th_dist", 0.02);
    this->declare_parameter<double>("pw_th_seeds", 0.02);
    this->declare_parameter<double>("pw_th_dist_v", 0.1);
    this->declare_parameter<double>("pw_th_seeds_v", 0.02);
    this->declare_parameter<double>("pw_min_range", 0.5);
    this->declare_parameter<double>("pw_uprightness_thr", 0.707);
    this->declare_parameter<bool>("pw_enable_RNR", true); 
    this->declare_parameter<bool>("pw_enable_TGR", true);

    // Clusterers
    this->declare_parameter<double>("euclidean_tolerance", 0.35);
    this->declare_parameter<double>("dbscan_eps", 0.30);
    this->declare_parameter<int>("dbscan_min_pts", 3);
    this->declare_parameter<int>("hdbscan_min_pts", 5);
    this->declare_parameter<double>("hdbscan_alpha", 0.02);
    this->declare_parameter<double>("voxel_grid_size", 0.02);
    this->declare_parameter<double>("depth_theta_thr", 10.0);
    this->declare_parameter<int>("depth_num_rings", 32);
    this->declare_parameter<double>("grid_resolution", 0.12);

    // PCA Feature thresholds
    this->declare_parameter<double>("pca_max_linearity", 0.88);
    this->declare_parameter<double>("pca_max_planarity", 0.8);
    this->declare_parameter<double>("pca_min_scatter", 0.02);
    this->declare_parameter<double>("pca_min_verticality", 0.65);
    
    // Estimator Rules
    this->declare_parameter<double>("rule_min_height", 0.10);
    this->declare_parameter<double>("rule_max_height", 0.80);
    this->declare_parameter<double>("rule_base_min_width", 0.10);
    this->declare_parameter<double>("rule_max_width", 0.50);
    this->declare_parameter<double>("rule_dynamic_width_decay", 0.005);
    this->declare_parameter<int>("rule_min_points_at_10m", 5);
    this->declare_parameter<int>("rule_min_points_cap", 60);
    this->declare_parameter<double>("rule_min_intensity", 5.0);

    // Merging
    this->declare_parameter<double>("merge_dist", 0.25);
    this->declare_parameter<double>("tracking_match_dist", 0.45);

    // Deskewing
    this->declare_parameter<bool>("use_deskewing", true);
    this->declare_parameter<std::string>("imu_topic", "/zed/zed_node/imu/data");
    this->declare_parameter<std::string>("imu_frame", "zed_imu_link");
    this->declare_parameter<double>("imu_lowpass_cutoff", 15.0);
    this->declare_parameter<std::vector<double>>("extrinsic_rotation", {0.999743, 0.0226629, 7.2829e-10, 8.06016e-10, -3.42052e-09, -1.0, -0.0226629, 0.999743, -3.43791e-09});
    this->declare_parameter<std::vector<double>>("extrinsic_translation", {0.0543494, -0.0235914, -0.0488917});
    this->declare_parameter<double>("roll_deg", -0.4);
    this->declare_parameter<double>("pitch_deg", 0.0);
    this->declare_parameter<double>("yaw_deg", 0.0);
    
    rcl_interfaces::msg::ParameterDescriptor world_up_axis_desc;
    world_up_axis_desc.dynamic_typing = true;
    this->declare_parameter("world_up_axis", rclcpp::ParameterValue(std::string("y")), world_up_axis_desc);
}

void PerceptionNode::loadParametersToConfig() {
    config_.clustering_algorithm = this->get_parameter("clustering_algorithm").as_string();
    config_.ground_remover_type = this->get_parameter("ground_remover_type").as_string();
    config_.estimator_type = this->get_parameter("estimator_type").as_string();
    config_.log_dir = this->get_parameter("log_dir").as_string();
    config_.profile_name = this->get_parameter("profile_name").as_string();
    config_.log_clusters = this->get_parameter("log_clusters").as_bool();
    config_.log_all_clusters = this->get_parameter("log_all_clusters").as_bool();
    
    config_.sensor_z = this->get_parameter("sensor_z").as_double();
    config_.max_range = this->get_parameter("max_range").as_double();
    config_.min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    config_.max_cluster_size = this->get_parameter("max_cluster_size").as_int();
    config_.debug_pub_freq = this->get_parameter("debug_pub_freq").as_int();
    config_.debug = this->get_parameter("debug").as_bool();
    
    config_.use_voxel_filter = this->get_parameter("use_voxel_filter").as_bool();
    config_.voxel_size = this->get_parameter("voxel_size").as_double();
    
    config_.bin_local_threshold = this->get_parameter("bin_local_threshold").as_double();
    config_.bin_hard_cutoff = this->get_parameter("bin_hard_cutoff").as_double();
    config_.bin_segments = this->get_parameter("bin_segments").as_int();
    config_.bin_bins = this->get_parameter("bin_bins").as_int();
    
    config_.slope_max_slope = this->get_parameter("slope_max_slope").as_double();
    config_.slope_max_z_diff = this->get_parameter("slope_max_z_diff").as_double();
    config_.slope_initial_threshold = this->get_parameter("slope_initial_threshold").as_double();
    config_.slope_segments = this->get_parameter("slope_segments").as_int();
    
    config_.pw_num_iter = this->get_parameter("pw_num_iter").as_int();
    config_.pw_th_dist = this->get_parameter("pw_th_dist").as_double();
    config_.pw_th_seeds = this->get_parameter("pw_th_seeds").as_double();
    config_.pw_th_dist_v = this->get_parameter("pw_th_dist_v").as_double();
    config_.pw_th_seeds_v = this->get_parameter("pw_th_seeds_v").as_double();
    config_.pw_min_range = this->get_parameter("pw_min_range").as_double();
    config_.pw_uprightness_thr = this->get_parameter("pw_uprightness_thr").as_double();
    config_.pw_enable_RNR = this->get_parameter("pw_enable_RNR").as_bool();
    config_.pw_enable_TGR = this->get_parameter("pw_enable_TGR").as_bool();
    
    config_.euclidean_tolerance = this->get_parameter("euclidean_tolerance").as_double();
    config_.dbscan_eps = this->get_parameter("dbscan_eps").as_double();
    config_.dbscan_min_pts = this->get_parameter("dbscan_min_pts").as_int();
    config_.hdbscan_min_pts = this->get_parameter("hdbscan_min_pts").as_int();
    config_.hdbscan_alpha = this->get_parameter("hdbscan_alpha").as_double();
    config_.voxel_grid_size = this->get_parameter("voxel_grid_size").as_double();
    config_.depth_theta_thr = this->get_parameter("depth_theta_thr").as_double();
    config_.depth_num_rings = this->get_parameter("depth_num_rings").as_int();
    config_.grid_resolution = this->get_parameter("grid_resolution").as_double();
    
    config_.pca_max_linearity = this->get_parameter("pca_max_linearity").as_double();
    config_.pca_max_planarity = this->get_parameter("pca_max_planarity").as_double();
    config_.pca_min_scatter = this->get_parameter("pca_min_scatter").as_double();
    config_.pca_min_verticality = this->get_parameter("pca_min_verticality").as_double();
    
    config_.rule_min_height = this->get_parameter("rule_min_height").as_double();
    config_.rule_max_height = this->get_parameter("rule_max_height").as_double();
    config_.rule_base_min_width = this->get_parameter("rule_base_min_width").as_double();
    config_.rule_max_width = this->get_parameter("rule_max_width").as_double();
    config_.rule_dynamic_width_decay = this->get_parameter("rule_dynamic_width_decay").as_double();
    config_.rule_min_points_at_10m = this->get_parameter("rule_min_points_at_10m").as_int();
    config_.rule_min_points_cap = this->get_parameter("rule_min_points_cap").as_int();
    config_.rule_min_intensity = this->get_parameter("rule_min_intensity").as_double();
    
    config_.merge_dist = this->get_parameter("merge_dist").as_double();
    config_.tracking_match_dist = this->get_parameter("tracking_match_dist").as_double();
    
    config_.use_deskewing = this->get_parameter("use_deskewing").as_bool();
    config_.imu_topic = this->get_parameter("imu_topic").as_string();
    config_.imu_frame = this->get_parameter("imu_frame").as_string();
    config_.imu_lowpass_cutoff = this->get_parameter("imu_lowpass_cutoff").as_double();
    config_.extrinsic_rotation = this->get_parameter("extrinsic_rotation").as_double_array();
    config_.extrinsic_translation = this->get_parameter("extrinsic_translation").as_double_array();
    config_.roll_deg = this->get_parameter("roll_deg").as_double();
    config_.pitch_deg = this->get_parameter("pitch_deg").as_double();
    config_.yaw_deg = this->get_parameter("yaw_deg").as_double();
    
    auto world_up_axis_param = this->get_parameter("world_up_axis");
    if (world_up_axis_param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        config_.world_up_axis = world_up_axis_param.as_bool() ? "y" : "n";
    } else if (world_up_axis_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        config_.world_up_axis = world_up_axis_param.as_string();
    }
}

void PerceptionNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!msg) return;

    frame_counter_++;
    if (profiler_) profiler_->startFrame();

    // 1. Clock Alignment
    if (clock_aligner_) {
        uint64_t pc_recv_ts_ns = this->get_clock()->now().nanoseconds();
        uint64_t target_ts_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        clock_aligner_->updateLidarTimeDynamic(target_ts_ns, pc_recv_ts_ns);
        uint64_t target_ts_aligned_ns = clock_aligner_->alignTimestamp(target_ts_ns);
        msg->header.stamp = rclcpp::Time(static_cast<int64_t>(target_ts_aligned_ns));
    }

    // 2. Data Conversion & Pre-processing (Point Timestamp Extraction & Truncation)
    if (!convertAndPreprocess(msg)) return;

    if (profiler_) profiler_->stopTimer("conversion");

    if (raw_cloud_ptr_->empty()) return;

    // 3. Delegate to PerceptionPipeline Facade
    obstacles_str_->clear();
    ground_str_->clear();
    std::vector<PointCloudPtr> merged_clusters;
    uint64_t target_ts_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

    std::vector<Cone> final_cones = pipeline_->run(
        raw_cloud_ptr_,
        target_ts_ns,
        msg->header.frame_id,
        frame_counter_,
        config_,
        cluster_logger_.get(),
        profiler_.get(),
        obstacles_str_,
        ground_str_,
        merged_clusters
    );

    // 4. Forward results to Active Object Visualizer
    if (viz_bridge_) {
        viz_bridge_->enqueue(msg->header, raw_cloud_ptr_, merged_clusters, final_cones, config_);
    }

    // 5. Diagnostics
    RCLCPP_INFO(this->get_logger(), 
        "Pipeline Diagnostic | Raw Points: %zu | Obstacles: %zu | Merged Clusters: %zu | Final Cones: %zu",
        raw_cloud_ptr_->size(), obstacles_str_->size(), merged_clusters.size(), final_cones.size());

    if (profiler_) {
        profiler_->endFrame(final_cones.size());

        auto total_ms = profiler_->getLastFrameTotalMs();
        RCLCPP_INFO(this->get_logger(), "Frame: %d | Latency: %.2f ms | Cones: %zu", 
            frame_counter_, total_ms, final_cones.size());
        if (total_ms > 20.0) {
            RCLCPP_WARN(this->get_logger(), "Performance Violation: Frame latency (%.2f ms) exceeds 20ms budget.", total_ms);
        }
    }
}

bool PerceptionNode::convertAndPreprocess(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    if (profiler_) profiler_->startTimer("conversion");
    
    try {
        pcl::fromROSMsg(*msg, *raw_cloud_ptr_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "ROS-to-PCL conversion failure: %s", e.what());
        if (profiler_) profiler_->stopTimer("conversion");
        return false;
    }
    
    if (raw_cloud_ptr_->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received null or empty point cloud.");
        if (profiler_) profiler_->stopTimer("conversion");
        return false;
    }

    // Robust point timestamp extraction
    int ts_offset = -1;
    uint8_t ts_datatype = 0;
    for (const auto& field : msg->fields) {
        if (field.name == "timestamp" || field.name == "time" || field.name == "t") {
            ts_offset = field.offset;
            ts_datatype = field.datatype;
            break;
        }
    }
    
    uint64_t target_ts_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (ts_offset >= 0) {
        int points_num = raw_cloud_ptr_->points.size();
        const uint8_t* raw_data_ptr = msg->data.data();
        
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < points_num; ++i) {
            auto& p = raw_cloud_ptr_->points[i];
            const uint8_t* pt_raw = raw_data_ptr + i * msg->point_step;
            double pt_time = 0.0;
            if (ts_datatype == sensor_msgs::msg::PointField::FLOAT32) {
                pt_time = static_cast<double>(*reinterpret_cast<const float*>(pt_raw + ts_offset));
            } else if (ts_datatype == sensor_msgs::msg::PointField::FLOAT64) {
                pt_time = *reinterpret_cast<const double*>(pt_raw + ts_offset);
            } else if (ts_datatype == sensor_msgs::msg::PointField::UINT32) {
                pt_time = static_cast<double>(*reinterpret_cast<const uint32_t*>(pt_raw + ts_offset));
            }
            
            double pt_abs_sec = 0.0;
            if (pt_time > 1e9) {
                if (pt_time > 1e15) {
                    pt_abs_sec = pt_time / 1e9;
                } else {
                    pt_abs_sec = pt_time;
                }
                
                if (clock_aligner_) {
                    uint64_t pt_abs_ns = static_cast<uint64_t>(pt_abs_sec * 1e9);
                    uint64_t pt_aligned_ns = clock_aligner_->alignTimestamp(pt_abs_ns);
                    pt_abs_sec = static_cast<double>(pt_aligned_ns) / 1e9;
                }
            } else {
                pt_abs_sec = (static_cast<double>(target_ts_ns) / 1e9) + pt_time;
            }
            p.timestamp = pt_abs_sec;
        }
    } else {
        double target_ts_sec = static_cast<double>(target_ts_ns) / 1e9;
        int points_num = raw_cloud_ptr_->points.size();
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < points_num; ++i) {
            raw_cloud_ptr_->points[i].timestamp = target_ts_sec;
        }
    }

    // Call internal preProcess helper in pipeline (for truncation)
    pipeline_->preProcess(raw_cloud_ptr_, config_);

    // Intensity distance normalization
    const float REF_DIST_SQ_INV = 1.0f / (10.0f * 10.0f);
    for (auto& pt : raw_cloud_ptr_->points) {
        float r2 = pt.x*pt.x + pt.y*pt.y + pt.z*pt.z;
        if (r2 > 1.0f) {
            float factor = std::max(1.0f, r2 * REF_DIST_SQ_INV);
            pt.intensity = std::min(255.0f, pt.intensity * factor); 
        }
    }

    return true;
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

} // namespace lidar_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_perception::PerceptionNode)
