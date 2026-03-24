#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <deque>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "utils/types.hpp"

namespace fs_perception {

/**
 * @class Deskewer
 * @brief Handles LiDAR point cloud deskewing using IMU data.
 * 
 * Corrects distortion caused by sensor motion during a single LiDAR sweep
 * by interpolating IMU-derived rotation and (optionally) translation.
 */
class Deskewer {
public:
    struct Config {
        std::string lidar_frame = "hesai_lidar";
        std::string imu_frame = "zed_imu_link";
        bool use_translation = true; 
        
        // Static rotation from Lidar to IMU (R_LI)
        Eigen::Quaternionf static_lidar_to_imu = Eigen::Quaternionf::Identity();

        // Static translation from IMU to Lidar (m) - The lever arm
        Eigen::Vector3f static_imu_to_lidar = Eigen::Vector3f::Zero();
    };

    explicit Deskewer(const Config& config);

    /**
     * @brief Adds an IMU message and pre-processes its orientation into the Lidar frame.
     * @param msg The original IMU message from the ZED.
     */
    void addImuMessage(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Deskews a point cloud using the pre-transformed buffer.
     * @param cloud Point cloud to correct.
     * @return True if successful.
     */
    bool deskew(PointCloudPtr cloud);

private:
    Config config_;
    
    // Structure to store pre-processed IMU data already in Lidar frame
    struct LidarOrientations {
        double timestamp;
        Eigen::Quaternionf orientation;
    };
    std::deque<LidarOrientations> imu_buffer_;
    std::mutex buffer_mutex_;

    /**
     * @brief Interpolates orientation from the pre-processed buffer.
     */
    bool getInterpolatedOrientation(double timestamp, Eigen::Quaternionf& orientation);

    // TODO: Add TF handling if Lidar and IMU are not aligned.
    // For now, we assume the IMU data is already in or can be easily mapped to the Lidar frame
    // or we only care about the relative rotation during the sweep.
};

} // namespace fs_perception
