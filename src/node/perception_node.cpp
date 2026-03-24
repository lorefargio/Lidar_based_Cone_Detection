#include "utils/types.hpp"

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
#include <chrono>
#include <memory>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

#include "filtering/ground_remover_interface.hpp"
#include "filtering/bin_based_ground_remover.hpp"
#include "filtering/slope_based_ground_remover.hpp"
#include "clustering/clusterer_interface.hpp"
#include "clustering/string_clusterer.hpp"
#include "clustering/euclidean_clusterer.hpp"
#include "clustering/grid_clusterer.hpp"
#include "clustering/dbscan_clusterer.hpp"
#include "clustering/adaptive_dbscan_clusterer.hpp"
#include "clustering/voxel_connected_components.hpp"
#include "estimation/estimator_interface.hpp"
#include "estimation/rule_based_estimator.hpp"
#include "estimation/model_fitting_estimator.hpp"
#include "utils/performance_profiler.hpp"
#include "utils/deskewer.hpp"

using namespace std::chrono_literals;

// Template instantiations for custom PCL point type
namespace pcl {
  template class PCLBase<fs_perception::PointT>;
  template class VoxelGrid<fs_perception::PointT>;
}

/**
 * @class LidarPerceptionNode
 * @brief Main ROS 2 node that manages the entire LiDAR perception pipeline.
 * 
 * Orchestrates ground removal, clustering, and cone estimation. 
 * Supports dynamic configuration of algorithms and parameters.
 */
class LidarPerceptionNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for LidarPerceptionNode.
     * Initializes all parameters, publishers, and subscribers.
     */
    LidarPerceptionNode() : Node("lidar_perception_node") {
        // --- Mandatory Bag Parameter Check ---
        this->declare_parameter<std::string>("bag_path", "");
        std::string bag_path = this->get_parameter("bag_path").as_string();
        if (bag_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "FATAL: 'bag_path' parameter is missing! The node cannot start without a valid bag.");
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO(this->get_logger(), "Processing bag: %s", bag_path.c_str());

        // --- Algorithm selection parameters ---
        this->declare_parameter<std::string>("clustering_algorithm", "grid");
        std::string algo = this->get_parameter("clustering_algorithm").as_string();

        // Factory for clustering algorithms
        if (algo == "euclidean") {
            clusterer_ = std::make_unique<fs_perception::EuclideanClusterer>(0.35f, 3, 300);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: EUCLIDEAN (KD-Tree)");
        } else if (algo == "string") {
            clusterer_ = std::make_unique<fs_perception::StringClusterer>();
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: STRING (Linear)");
        } else if (algo == "dbscan") {
            clusterer_ = std::make_unique<fs_perception::DBSCANClusterer>(0.30f, 3, 3, 500);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: DBSCAN (KD-Tree)");
        } else if (algo == "hdbscan") {
            clusterer_ = std::make_unique<fs_perception::AdaptiveDBSCANClusterer>(0.15f, 0.015f, 3, 3, 500);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: ADAPTIVE-DBSCAN (Distance Scaled)");
        } else if (algo == "voxel") {
            clusterer_ = std::make_unique<fs_perception::VoxelConnectedComponents>(0.15f, 3, 500);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: VOXEL CONNECTED COMPONENTS (3D Grid)");
        } else {
            clusterer_ = std::make_unique<fs_perception::GridClusterer>(0.20f, 25.0f, 3, 300);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: GRID (2D Connected Components)");
        }

        // Factory for ground removal algorithms
        this->declare_parameter<std::string>("ground_remover_type", "slope_based");
        std::string gr_type = this->get_parameter("ground_remover_type").as_string();

        if (gr_type == "bin_based") {
            ground_remover_ = std::make_unique<fs_perception::BinBasedGroundRemover>();
            RCLCPP_INFO(this->get_logger(), "Ground Removal: BIN-BASED (Fast)");
        } else if (gr_type == "slope_based") {
            ground_remover_ = std::make_unique<fs_perception::SlopeBasedGroundRemover>();
            RCLCPP_INFO(this->get_logger(), "Ground Removal: SLOPE-BASED (Precise)");
        } else {
            ground_remover_ = std::make_unique<fs_perception::SlopeBasedGroundRemover>();
            RCLCPP_INFO(this->get_logger(), "Ground Removal: Defaulting to SLOPE-BASED");
        }

        // Post-filtering parameters (Downsampling)
        this->declare_parameter<bool>("use_voxel_filter", false);
        this->declare_parameter<double>("voxel_size", 0.05);

        // Deskewing parameters
        this->declare_parameter<bool>("use_deskewing", true);
        this->declare_parameter<std::string>("imu_topic", "/zed/zed_node/imu/data");

        if (this->get_parameter("use_deskewing").as_bool()) {
            fs_perception::Deskewer::Config deskewer_cfg;
            deskewer_ = std::make_unique<fs_perception::Deskewer>(deskewer_cfg);
            RCLCPP_INFO(this->get_logger(), "Lidar Deskewing ENABLED.");
        }

        // Factory for estimation algorithms
        this->declare_parameter<std::string>("estimator_type", "rule_based");
        std::string est_type = this->get_parameter("estimator_type").as_string();

        // Estimation parameters (PCA & Geometric rules)
        this->declare_parameter<double>("dynamic_width_decay", 0.005);
        this->declare_parameter<int>("min_points_at_10m", 10);
        this->declare_parameter<double>("pca_max_linearity", 0.8);
        this->declare_parameter<double>("pca_max_planarity", 0.8);
        this->declare_parameter<double>("pca_min_scatter", 0.02); // Keep optimized value

        if (est_type == "rule_based") {
            fs_perception::RuleBasedEstimator::Config config;
            config.dynamic_width_decay = this->get_parameter("dynamic_width_decay").as_double();
            config.min_points_at_10m = this->get_parameter("min_points_at_10m").as_int();
            config.max_linearity = this->get_parameter("pca_max_linearity").as_double();
            config.max_planarity = this->get_parameter("pca_max_planarity").as_double();
            config.min_scatter = this->get_parameter("pca_min_scatter").as_double();

            estimator_ = std::make_unique<fs_perception::RuleBasedEstimator>(config);
            RCLCPP_INFO(this->get_logger(), "Estimator Algorithm: RULE-BASED (Geometric Classification)");
        } else if (est_type == "ransac") {
            estimator_ = std::make_unique<fs_perception::ModelFittingEstimator>();
            RCLCPP_INFO(this->get_logger(), "Estimator Algorithm: RANSAC MODEL FITTING (Cylinder)");
        } else {
            estimator_ = std::make_unique<fs_perception::RuleBasedEstimator>();
            RCLCPP_INFO(this->get_logger(), "Estimator Algorithm: Defaulting to RULE-BASED");
        }

        // Benchmarking Setup
        this->declare_parameter<std::string>("log_dir", "log_profiler/");
        std::string log_dir = this->get_parameter("log_dir").as_string();
        
        // Ensure log directory ends with a slash
        if (!log_dir.empty() && log_dir.back() != '/') log_dir += '/';
        
        try {
            std::filesystem::create_directories(log_dir); 
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create log directory %s: %s", log_dir.c_str(), e.what());
        }
        
        profiler_ = std::make_unique<fs_perception::PerformanceProfiler>(algo);
        json_file_path_ = log_dir + "profiler_" + algo + ".json";
        RCLCPP_INFO(this->get_logger(), "Profiler initialized. Will save to: %s", json_file_path_.c_str());

        // --- ROS Infrastructure (Pub/Sub) ---
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", qos, 
            std::bind(&LidarPerceptionNode::callback, this, std::placeholders::_1));

        if (this->get_parameter("use_deskewing").as_bool()) {
            std::string imu_topic = this->get_parameter("imu_topic").as_string();
            sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, 100, 
                [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                    if (deskewer_) deskewer_->addImuMessage(msg);
                });
            RCLCPP_INFO(this->get_logger(), "Subscribed to IMU: %s", imu_topic.c_str());
        }

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/cones_vis", 10);
        pub_cones_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/cones", 10);
        pub_cone_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/cone_points", 10);
        pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_debug", 10);
        pub_no_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/no_ground", 10);

        RCLCPP_INFO(this->get_logger(), "Perception Node Started.");
    }

    /**
     * @brief Destructor for LidarPerceptionNode.
     * Ensures all benchmarking profiling data is flushed to disk on shutdown.
     */
    ~LidarPerceptionNode() {
        if (profiler_) {
            profiler_->saveToJSON(json_file_path_);
            RCLCPP_INFO(this->get_logger(), "Profiling data saved to JSON before exit.");
        }
    }

private:
    /**
     * @brief Point cloud processing callback function.
     * Executes the entire perception pipeline frame-by-frame.
     * @param msg Incoming PointCloud2 message from the LiDAR sensor.
     */
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (!msg) return;

        frame_counter_++;
        if (profiler_) profiler_->startFrame();

        // Phase 1: Point cloud conversion from ROS to PCL
        fs_perception::PointCloudPtr raw_cloud(new fs_perception::PointCloud);
        try {
            pcl::fromROSMsg(*msg, *raw_cloud);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Conversion from ROS to PCL failed: %s", e.what());
            return;
        }
        
        if (raw_cloud->empty()) return;

        // Phase 1.1: Lidar Deskewing
        if (this->get_parameter("use_deskewing").as_bool() && deskewer_) {
            if (profiler_) profiler_->startTimer("deskewing");
            if (!deskewer_->deskew(raw_cloud)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                    "Deskewing failed (possibly waiting for IMU data).");
            }
            if (profiler_) profiler_->stopTimer("deskewing");
        }

        // Phase 2: Ground Removal (Run on full resolution for maximum precision)
        if (!ground_remover_) {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Ground remover not initialized!");
            return;
        }

        if (profiler_) profiler_->startTimer("ground_removal");
        obstacles_str_->clear();
        ground_str_->clear();

        ground_remover_->removeGround(raw_cloud, obstacles_str_, ground_str_);
        if (profiler_) profiler_->stopTimer("ground_removal");

        // Phase 2.1: Optional Voxel Grid Downsampling to speed up subsequent stages
        if (this->get_parameter("use_voxel_filter").as_bool()) {
            double leaf_size = this->get_parameter("voxel_size").as_double();
            
            // Safety Check: Leaf size cannot be smaller than 0.01m to avoid memory explosion
            if (leaf_size < 0.01) leaf_size = 0.01;

            pcl::VoxelGrid<fs_perception::PointT> voxel_grid;
            voxel_grid.setInputCloud(obstacles_str_);
            voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
            
            fs_perception::PointCloudPtr filtered_obstacles(new fs_perception::PointCloud);
            try {
                voxel_grid.filter(*filtered_obstacles);
                obstacles_str_ = filtered_obstacles;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "VoxelGrid failed: %s", e.what());
            }
        }

        // Publish debug ground clouds
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_str_, ground_msg);
        ground_msg.header = msg->header;
        pub_ground_->publish(ground_msg);

        sensor_msgs::msg::PointCloud2 no_ground_msg;
        pcl::toROSMsg(*obstacles_str_, no_ground_msg);
        no_ground_msg.header = msg->header;
        pub_no_ground_->publish(no_ground_msg);

        // Phase 3: Object Clustering
        if (!clusterer_) {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Clusterer not initialized!");
            return;
        }

        if (profiler_) profiler_->startTimer("clustering");
        std::vector<fs_perception::PointCloudPtr> clusters;
        clusterer_->cluster(obstacles_str_, clusters);

        // Phase 3.1: Proximity-based Cluster Merging to handle over-segmented objects
        std::vector<fs_perception::PointCloudPtr> merged_clusters;
        std::vector<bool> merged_flag(clusters.size(), false);
        
        // Centroids with aligned_allocator for Eigen SSE compatibility
        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids(clusters.size());
        
        for(size_t i = 0; i < clusters.size(); ++i) {
            pcl::compute3DCentroid(*clusters[i], centroids[i]);
        }

        const float MERGE_DIST_SQ = 0.25f * 0.25f; // 25cm distance for merging clusters
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (merged_flag[i]) continue;
            fs_perception::PointCloudPtr super_cluster(new fs_perception::PointCloud(*clusters[i]));

            for (size_t j = i + 1; j < clusters.size(); ++j) {
                if (merged_flag[j]) continue;
                float dx = centroids[i][0] - centroids[j][0];
                float dy = centroids[i][1] - centroids[j][1];

                if (dx*dx + dy*dy < MERGE_DIST_SQ) {
                    *super_cluster += *clusters[j];
                    merged_flag[j] = true; 
                }
            }
            merged_clusters.push_back(super_cluster);
        }
        if (profiler_) profiler_->stopTimer("clustering");

        // Phase 4: Candidate Estimation and Duplicate Filtering
        if (!estimator_) {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Estimator not initialized!");
            return;
        }

        if (profiler_) profiler_->startTimer("estimation");
        std::vector<fs_perception::Cone> candidate_cones;

        for (const auto& c : merged_clusters) {
            auto cone = estimator_->estimate(c);
            if (cone.confidence > 0.5f) {
                candidate_cones.push_back(cone);
            }
        }
        
        // Duplicate suppression using spatial proximity
        const float MIN_DIST_SQ = 0.4f * 0.4f; // 40cm duplicate radius
        std::vector<fs_perception::Cone> final_cones;

        for (const auto& candidate : candidate_cones) {
            bool is_duplicate = false;
            for (const auto& accepted : final_cones) {
                float dist_sq = (candidate.x - accepted.x)*(candidate.x - accepted.x) +
                                (candidate.y - accepted.y)*(candidate.y - accepted.y);
                if (dist_sq < MIN_DIST_SQ) {
                    is_duplicate = true;
                    break;
                }
            }
            if (!is_duplicate) {
                final_cones.push_back(candidate);
            }
        }

        if (profiler_) profiler_->stopTimer("estimation");

        // Phase 5: Visualization Markers and Detection Output
        visualization_msgs::msg::MarkerArray markers;
        fs_perception::PointCloud cones_cloud_out; 
        fs_perception::PointCloud cone_points_out;

        // Clear previous frame markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.header = msg->header;
        delete_marker.ns = "cones";
        delete_marker.id = 0;
        markers.markers.push_back(delete_marker);

        // Clear distance labels as well
        visualization_msgs::msg::Marker delete_labels;
        delete_labels.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_labels.header = msg->header;
        delete_labels.ns = "distance_labels";
        delete_labels.id = 0;
        markers.markers.push_back(delete_labels);

        int id_counter = 1;
        for (const auto& cone : final_cones) {
            // 1. Geometry Marker (Cylinder)
            visualization_msgs::msg::Marker m;
            m.header = msg->header;
            m.ns = "cones";
            m.id = id_counter;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = rclcpp::Duration::from_seconds(0.2); 

            m.pose.position.x = cone.x;
            m.pose.position.y = cone.y;
            m.pose.position.z = cone.z + (cone.height / 2.0); 
            m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = cone.height;
            m.color.a = 1.0;
            
            // Color mapping for visualization
            if (cone.color == fs_perception::ConeColor::BLUE) { 
                m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f; 
            } else if (cone.color == fs_perception::ConeColor::YELLOW) { 
                m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; 
            } else { 
                m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f; 
            }

            markers.markers.push_back(m);

            // 2. Distance Label Marker (Text)
            visualization_msgs::msg::Marker t;
            t.header = msg->header;
            t.ns = "distance_labels";
            t.id = id_counter++;
            t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            t.action = visualization_msgs::msg::Marker::ADD;
            t.lifetime = rclcpp::Duration::from_seconds(0.2);
            t.pose.position.x = cone.x;
            t.pose.position.y = cone.y;
            t.pose.position.z = cone.z + cone.height + 0.2;
            t.scale.z = 0.15; // text height
            t.color.a = 1.0; t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0;
            
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << cone.range << "m";
            t.text = ss.str();
            markers.markers.push_back(t);

            // 3. Centroid Output (For Range/Bearing information)
            fs_perception::PointT p_out;
            p_out.x = cone.x; p_out.y = cone.y; p_out.z = cone.z;
            p_out.intensity = cone.range; // Using intensity as a way to pass range in default cloud
            cones_cloud_out.push_back(p_out);

            // 4. Full Points Output (For Bounding Box check)
            if (cone.cloud) {
                cone_points_out += *cone.cloud;
            }
        }

        pub_markers_->publish(markers);

        sensor_msgs::msg::PointCloud2 cones_msg;
        pcl::toROSMsg(cones_cloud_out, cones_msg);
        cones_msg.header = msg->header;
        pub_cones_->publish(cones_msg);

        sensor_msgs::msg::PointCloud2 cone_points_msg;
        pcl::toROSMsg(cone_points_out, cone_points_msg);
        cone_points_msg.header = msg->header;
        pub_cone_points_->publish(cone_points_msg);

        // Phase 6: Finalize frame profiling
        if (profiler_) {
            profiler_->endFrame(final_cones.size());

            if (frame_counter_ % 50 == 0) {
                auto total_ms = profiler_->getLastFrameTotalMs();
                RCLCPP_INFO(this->get_logger(), "Frame: %d | Total Latency: %.2f ms | Cones: %zu", 
                    frame_counter_, total_ms, final_cones.size());
                
                if (total_ms > 50.0) {
                    RCLCPP_WARN(this->get_logger(), "PERFORMANCE ALERT: Latency (%.2f ms) exceeds 20Hz budget!", total_ms);
                }
            }
        }
    }

    // ROS 2 publishers and subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cone_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_;

    // Internal pipeline storage and interfaces
    fs_perception::PointCloudPtr obstacles_str_{new fs_perception::PointCloud};
    fs_perception::PointCloudPtr ground_str_{new fs_perception::PointCloud};
    std::unique_ptr<fs_perception::GroundRemoverInterface> ground_remover_;
    std::unique_ptr<fs_perception::ClustererInterface> clusterer_;
    std::unique_ptr<fs_perception::EstimatorInterface> estimator_;
    std::unique_ptr<fs_perception::Deskewer> deskewer_;
    
    // Benchmarking and logging tools
    std::unique_ptr<fs_perception::PerformanceProfiler> profiler_;
    std::string json_file_path_;
    int frame_counter_ = 0;
};

/**
 * @brief Entry point for the lidar perception node.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
