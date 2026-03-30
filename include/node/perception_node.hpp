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
 * @brief Main ROS 2 node that manages the entire LiDAR perception pipeline.
 * 
 * Orchestrates ground removal, clustering, and cone estimation. 
 * Supports dynamic configuration of algorithms and parameters.
 */
class LidarPerceptionNode : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /**
     * @brief Constructor for LidarPerceptionNode.
     * Initializes all parameters, publishers, and subscribers.
     */
    LidarPerceptionNode();

    /**
     * @brief Destructor for LidarPerceptionNode.
     * Ensures all benchmarking profiling data is flushed to disk on shutdown.
     */
    ~LidarPerceptionNode() override;

private:
    /**
     * @brief Point cloud processing callback function.
     * Executes the entire perception pipeline frame-by-frame.
     * @param msg Incoming PointCloud2 message from the LiDAR sensor.
     */
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Early point cloud truncation and sanitization.
     */
    void preProcess(PointCloudPtr cloud);

    // ROS 2 publishers and subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cone_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_;

    // Internal pipeline storage and interfaces
    PointCloudPtr raw_cloud_ptr_{new PointCloud};
    PointCloudPtr obstacles_str_{new PointCloud};
    PointCloudPtr ground_str_{new PointCloud};
    std::unique_ptr<GroundRemoverInterface> ground_remover_;
    std::unique_ptr<ClustererInterface> clusterer_;
    std::unique_ptr<ConeEstimator> estimator_;
    std::unique_ptr<Deskewer> deskewer_;
    
    // Benchmarking and logging tools
    std::unique_ptr<PerformanceProfiler> profiler_;
    std::unique_ptr<ClusterLogger> cluster_logger_;
    std::string json_file_path_;
    std::string csv_file_path_;
    int frame_counter_ = 0;
};

} // namespace fs_perception
