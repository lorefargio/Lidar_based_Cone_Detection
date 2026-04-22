/**
 * @file perception_node.hpp
 * @brief Principal ROS 2 node orchestrating the LiDAR-based perception pipeline.
 * 
 * This module integrates several stages: point cloud pre-processing (including early voxelization),
 * ground removal, object clustering via various spatial partition strategies, and 
 * rule-based cone estimation. It leverages high-performance ROS 2 primitives such as 
 * SensorDataQoS and efficient memory management using std::move to minimize latency.
 * 
 * @author Gemini CLI (Autonomous Agent)
 * @date 2026-03-30
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
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
#include "utils/deskewer.hpp"
#include "utils/cluster_logger.hpp"

namespace fs_perception {

/**
 * @class LidarPerceptionNode
 * @brief Central controller for the LiDAR perception stack.
 * 
 * The LidarPerceptionNode manages the transformation of raw sensor data into semantic 
 * cone detections. It implements a multi-stage pipeline:
 * 1.  **Pre-processing:** Fast distance truncation and early voxelization for noise reduction.
 * 2.  **Deskewing:** Compensation for ego-motion using IMU integration.
 * 3.  **Ground Removal:** Separation of traversable surface points from obstacles.
 * 4.  **Clustering:** Spatial grouping of non-ground points into discrete candidates.
 * 5.  **Estimation:** Geometric and statistical validation of candidates using PCA-derived features.
 * 
 * Optimized for low-latency execution through the use of rclcpp::SensorDataQoS and 
 * asynchronous message passing with std::move.
 */
class LidarPerceptionNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief Constructs the LidarPerceptionNode.
     * 
     * Performs parameter declaration, initializes modular pipeline components (ground removers, 
     * clusterers, estimators), and establishes communication interfaces with SensorDataQoS.
     */
    LidarPerceptionNode();

    /**
     * @brief Finalizes the LidarPerceptionNode.
     * 
     * Ensures all persistence operations, such as flushing performance profiling data to 
     * JSON and cluster characteristics to CSV, are completed before shutdown.
     */
    ~LidarPerceptionNode() override;

private:
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
    void preProcess(PointCloudPtr cloud);

    // ROS 2 communication interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_; ///< LiDAR point cloud subscriber.
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;          ///< IMU subscriber for deskewing.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_; ///< Visualization marker publisher.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_slam_input_; ///< Dedicated SLAM input publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_;   ///< Cone centroids publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_slam_cones_; ///< SLAM-specific cone coordinates publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cone_points_; ///< Raw cone points publisher.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;  ///< Ground points publisher (debug).
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_; ///< Obstacle points publisher (debug).

    // Core pipeline components
    PointCloudPtr raw_cloud_ptr_{new PointCloud};      ///< Buffer for raw input cloud.
    PointCloudPtr obstacles_str_{new PointCloud};      ///< Buffer for non-ground points.
    PointCloudPtr ground_str_{new PointCloud};         ///< Buffer for ground points.
    std::unique_ptr<GroundRemoverInterface> ground_remover_; ///< Abstract ground removal interface.
    std::unique_ptr<ClustererInterface> clusterer_;          ///< Abstract clustering interface.
    std::unique_ptr<ConeEstimator> estimator_;               ///< Geometric cone classification engine.
    std::unique_ptr<Deskewer> deskewer_;                     ///< Lidar motion compensation module.
    
    // Benchmarking and diagnostics
    std::unique_ptr<PerformanceProfiler> profiler_; ///< Real-time latency tracking.
    std::unique_ptr<ClusterLogger> cluster_logger_; ///< Offline feature analysis logger.
    std::string json_file_path_;                    ///< Destination path for profiling reports.
    std::string csv_file_path_;                     ///< Destination path for cluster logs.
    int frame_counter_ = 0;                         ///< Incremental frame index.
};

} // namespace fs_perception
