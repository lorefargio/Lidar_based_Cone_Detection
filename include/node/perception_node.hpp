#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <string>
#include <vector>

#include "utils/types.hpp"
#include "node/pipeline_config.hpp"
#include "node/perception_pipeline.hpp"
#include "node/visualization_bridge.hpp"
#include "utils/clock_aligner.hpp"
#include "utils/performance_profiler.hpp"
#include "utils/cluster_logger.hpp"

namespace lidar_perception {

/**
 * @class PerceptionNode
 * @brief Minimal ROS 2 wrapper orchestrating communication, parameter binding, and pipeline lifecycle.
 */
class PerceptionNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    explicit PerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PerceptionNode() override;

private:
    void initializeParameters();
    void loadParametersToConfig();
    bool convertAndPreprocess(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void saveConfig(const std::string& filepath);

    // ROS 2 communication interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

    // Core Pipeline Components
    PipelineConfig config_;
    std::unique_ptr<PerceptionPipeline> pipeline_;
    std::unique_ptr<VisualizationBridge> viz_bridge_;
    std::unique_ptr<fs_fusion::ClockAligner> clock_aligner_;

    // Main thread local buffers (passed down to the pipeline)
    PointCloudPtr raw_cloud_ptr_{new PointCloud};
    PointCloudPtr obstacles_str_{new PointCloud};
    PointCloudPtr ground_str_{new PointCloud};

    // Benchmarking and diagnostics
    std::unique_ptr<PerformanceProfiler> profiler_;
    std::unique_ptr<ClusterLogger> cluster_logger_;
    std::string json_file_path_;
    std::string csv_file_path_;
    std::string config_json_file_path_;
    int frame_counter_ = 0;
    bool debug_mode_ = false;
};

} // namespace lidar_perception
