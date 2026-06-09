#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "utils/types.hpp"
#include "node/pipeline_config.hpp"

namespace lidar_perception {

/**
 * @class VisualizationBridge
 * @brief Handles formatting and publishing of visualization topics asynchronously on a dedicated worker thread.
 */
class VisualizationBridge {
public:
    explicit VisualizationBridge(rclcpp::Node* node);
    ~VisualizationBridge();

    /**
     * @brief Enqueues a frame of perception data for asynchronous visualization.
     */
    void enqueue(
        const std_msgs::msg::Header& header,
        const PointCloudPtr& processing_cloud,
        const std::vector<PointCloudPtr>& merged_clusters,
        const std::vector<Cone>& final_cones,
        const PipelineConfig& config);

    /**
     * @brief Stop the worker thread and wait for completion.
     */
    void stop();

private:
    void workerLoop();

    struct VizData {
        std_msgs::msg::Header header;
        PointCloudPtr cloud_viz;
        PointCloudPtr cones_cloud;
        PointCloudPtr cone_points;
        PointCloudPtr obstacles_cloud;
        std::unique_ptr<visualization_msgs::msg::MarkerArray> markers;
    };

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cone_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_viz_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;

    std::deque<std::unique_ptr<VizData>> queue_;
    std::thread worker_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_thread_ = false;
    bool debug_mode_ = true;
};

} // namespace lidar_perception
