/**
 * @file perception_node.hpp
 * @brief Principal ROS 2 node orchestrating the LiDAR-based perception pipeline.
 * 
 * This module integrates several stages: point cloud pre-processing (including early voxelization),
 * ground removal, object clustering via various spatial partition strategies, and 
 * rule-based object estimation. It leverages high-performance ROS 2 primitives such as 
 * SensorDataQoS and efficient memory management using std::unique_ptr and std::move to minimize latency. 
 *
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
#include <memory>
#include <string>
#include <vector>

#include "utils/types.hpp"
#include "filtering/ground_remover_interface.hpp"
#include "clustering/clusterer_interface.hpp"
#include "estimation/cone_estimator.hpp"
#include "utils/performance_profiler.hpp"
#include "utils/transform_manager.hpp"
#include "utils/imu_interpolator.hpp"
#include "utils/cluster_logger.hpp"
#include "utils/clock_aligner.hpp"

namespace lidar_perception {

/**
 * @class PerceptionNode
 * @brief Central controller for the LiDAR perception stack.
 * 
 * The PerceptionNode manages the transformation of raw sensor data into semantic 
 * object detections. It implements a multi-stage pipeline:
 * 1.  **Pre-processing:** Fast distance truncation and early voxelization for noise reduction.
 * 2.  **Deskewing:** Compensation for ego-motion using IMU integration.
 * 3.  **Ground Removal:** Separation of traversable surface points from obstacles.
 * 4.  **Clustering:** Spatial grouping of non-ground points into discrete candidates.
 * 5.  **Estimation:** Geometric and statistical validation of candidates using PCA-derived features.
 * 
 * Optimized for low-latency execution through the use of rclcpp::SensorDataQoS and 
 * asynchronous message passing with std::unique_ptr and std::move.
 */
class PerceptionNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief Constructs the PerceptionNode.
     * 
     * Performs parameter declaration, initializes modular pipeline components (ground removers, 
     * clusterers, estimators), and establishes communication interfaces with SensorDataQoS.
     */
    PerceptionNode();

    /**
     * @brief Finalizes the PerceptionNode.
     * 
     * Ensures all persistence operations, such as flushing performance profiling data to 
     * JSON and cluster characteristics to CSV, are completed before shutdown.
     */
    ~PerceptionNode() override;

private:
    // Node Initialization Helpers
    void initializeParameters();
    void initializeGroundRemover();
    void initializeClusterer();
    void initializeEstimator();
    void initializeDeskewing();

    // Pipeline Stage Helpers
    bool convertAndPreprocess(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void performDeskewing(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, PointCloudPtr& processing_cloud);
    void performClusteringAndMerging(const PointCloudPtr& obstacles_cloud, std::vector<PointCloudPtr>& merged_clusters);
    void estimateCones(const std::vector<PointCloudPtr>& merged_clusters, std::vector<Cone>& candidate_cones);
    void aggregateDuplicates(const std::vector<Cone>& candidate_cones, std::vector<Cone>& final_cones);
    void handoverToVisualization(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, const PointCloudPtr& processing_cloud, const std::vector<PointCloudPtr>& merged_clusters, const std::vector<Cone>& final_cones);

    /**
     * @brief Primary processing pipeline callback.
     * 
     * Orchestrates the sequential execution of the perception pipeline upon receipt of 
     * a new LiDAR point cloud frame.
     * 
     * @param msg The incoming sensor_msgs::msg::PointCloud2 message.
     */
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Initial spatial filtering and data sanitization.
     * 
     * Implements early truncation of points based on radial distance and removes 
     * non-finite coordinates. This stage is critical for preventing downstream 
     * algorithmic overflows and improving computational efficiency.
     * 
     * @param cloud Pointer to the point cloud to be processed in-place.
     */
    void preProcess(const PointCloudPtr& cloud);

    /**
     * @brief Worker thread for asynchronous visualization publication.
     * 
     * Handles PCL-to-ROS conversion and publication in a background thread 
     * to prevent blocking the real-time perception pipeline.
     */
    void visualizationWorker();

    /**
     * @brief Saves the current node configuration to a JSON file.
     * @param filepath Path to the output JSON file.
     */
    void saveConfig(const std::string& filepath);

    // ROS 2 communication interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_; ///< LiDAR point cloud subscriber.
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;          ///< IMU subscriber for deskewing.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_; ///< Visualization marker publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_;   ///< Object centroids publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cone_points_; ///< Raw object points publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_viz_;   ///< Lightweight original cloud for visualization.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;   ///< Ground-removed, deskewed obstacle points publisher.

    // Core pipeline components
    PointCloudPtr raw_cloud_ptr_{new PointCloud};      ///< Buffer for raw input cloud.
    PointCloudPtr filtered_cloud_ptr_{new PointCloud}; ///< Buffer for early-voxelized cloud.
    PointCloudPtr obstacles_str_{new PointCloud};      ///< Buffer for non-ground points.
    PointCloudPtr ground_str_{new PointCloud};         ///< Buffer for ground points.

    // Asynchronous Visualization Logic
    struct VizData {
        std_msgs::msg::Header header;
        PointCloudPtr cloud_viz;
        PointCloudPtr cones_cloud;
        PointCloudPtr cone_points;
        PointCloudPtr obstacles_cloud;
        std::unique_ptr<visualization_msgs::msg::MarkerArray> markers;
    };
    std::deque<std::unique_ptr<VizData>> viz_queue_;
    std::thread viz_thread_;
    std::mutex viz_mutex_;
    std::condition_variable viz_cv_;
    bool stop_viz_thread_ = false;

    std::unique_ptr<GroundRemoverInterface> ground_remover_; ///< Abstract ground removal interface.
    std::unique_ptr<ClustererInterface> clusterer_;          ///< Abstract clustering interface.
    std::unique_ptr<ConeEstimator> estimator_;               ///< Geometric object classification engine.
    std::unique_ptr<fs_fusion::TransformManager> tf_manager_;
    std::unique_ptr<fs_fusion::ImuInterpolator> imu_interpolator_;
    std::unique_ptr<fs_fusion::ClockAligner> clock_aligner_;
    
    Eigen::Vector3d g_world_; ///< World gravity vector.
    
    // Benchmarking and diagnostics
    std::unique_ptr<PerformanceProfiler> profiler_; ///< Real-time latency tracking.
    std::unique_ptr<ClusterLogger> cluster_logger_; ///< Offline feature analysis logger.
    std::string json_file_path_;                    ///< Destination path for profiling reports.
    std::string csv_file_path_;                     ///< Destination path for cluster logs.
    std::string config_json_file_path_;             ///< Destination path for configuration log.
    int frame_counter_ = 0;                         ///< Incremental frame index.
    bool debug_mode_ = false;                       ///< Toggle high-overhead debugging publications.
};

} // namespace lidar_perception
