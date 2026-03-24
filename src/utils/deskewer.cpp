#include "utils/deskewer.hpp"
#include <algorithm>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

namespace fs_perception {

Deskewer::Deskewer(const Config& config) : config_(config) {
}

void Deskewer::addImuMessage(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // 1. Convert message orientation to Eigen
    Eigen::Quaternionf q_imu(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    // 2. PRE-PROCESSING: Transform to Lidar Frame once (O(1) per IMU message)
    // q_lidar = q_imu * inv(R_LI)
    Eigen::Quaternionf q_lidar = q_imu * config_.static_lidar_to_imu.inverse();
    
    // 3. Store in the optimized buffer
    LidarOrientations data;
    data.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    data.orientation = q_lidar;
    
    imu_buffer_.push_back(data);

    // Increase buffer size to 2000 messages (~5 seconds at 400Hz) 
    // to handle bag jitter and processing delays.
    while (imu_buffer_.size() > 2000) {
        imu_buffer_.pop_front();
    }
}

bool Deskewer::deskew(PointCloudPtr cloud) {
    if (cloud->empty()) return false;

    // Check if the first point has a valid timestamp (not zero or near-zero)
    double sweep_start_time = cloud->points.front().timestamp;
    
    if (sweep_start_time < 1.0) { // Likely uninitialized or relative timestamp
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("deskewer"), steady_clock, 5000, 
            "Lidar points have invalid timestamps (approx 0). Check Lidar driver configuration.");
        return false;
    }

    Eigen::Quaternionf start_orientation;
    if (!getInterpolatedOrientation(sweep_start_time, start_orientation)) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        if (imu_buffer_.empty()) {
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("deskewer"), steady_clock, 5000, "IMU buffer is empty. Waiting for data...");
        } else {
            double buf_start = imu_buffer_.front().timestamp;
            double buf_end = imu_buffer_.back().timestamp;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("deskewer"), steady_clock, 5000, 
                "Lidar time (%.3f) out of IMU buffer range [%.3f - %.3f]. Drift: %.3f s", 
                sweep_start_time, buf_start, buf_end, sweep_start_time - buf_end);
        }
        return false;
    }

    Eigen::Quaternionf start_orientation_inv = start_orientation.inverse();
    Eigen::Vector3f lever_arm = config_.static_imu_to_lidar;

    // Process each point in the cloud (O(N) where N is points per sweep)
    for (auto& point : cloud->points) {
        Eigen::Quaternionf current_orientation;
        if (getInterpolatedOrientation(point.timestamp, current_orientation)) {
            // 1. Rotation Correction
            Eigen::Quaternionf relative_rotation = start_orientation_inv * current_orientation;

            Eigen::Vector3f p(point.x, point.y, point.z);
            Eigen::Vector3f p_corrected = relative_rotation * p;

            // 2. Lever Arm Compensation (Translation correction)
            // If the car rotates, the Lidar (at the end of the arm) moves linearly.
            // Delta_T = start_orientation_inv * (current_rotation * lever_arm - start_rotation * lever_arm)
            if (config_.use_translation && lever_arm.norm() > 0.001f) {
                Eigen::Vector3f p_start_lever = start_orientation * lever_arm;
                Eigen::Vector3f p_curr_lever = current_orientation * lever_arm;
                Eigen::Vector3f translation_compensation = start_orientation_inv * (p_curr_lever - p_start_lever);
                p_corrected += translation_compensation;
            }

            point.x = p_corrected.x();
            point.y = p_corrected.y();
            point.z = p_corrected.z();
        }
    }

    return true;
}

bool Deskewer::getInterpolatedOrientation(double timestamp, Eigen::Quaternionf& orientation) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (imu_buffer_.size() < 2) return false;

    // Buffer range check
    if (timestamp < imu_buffer_.front().timestamp || timestamp > imu_buffer_.back().timestamp) {
        return false;
    }

    // Binary search (std::lower_bound) is fast on deque of structs
    auto it = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), timestamp,
        [](const LidarOrientations& data, double ts) {
            return data.timestamp < ts;
        });

    if (it == imu_buffer_.begin() || it == imu_buffer_.end()) {
        return false;
    }

    const auto& data_after = *it;
    const auto& data_before = *(--it);

    // Diagnostics: Log time gap between IMU messages (should be ~2.5ms for 400Hz)
    double msg_gap = data_after.timestamp - data_before.timestamp;
    if (msg_gap > 0.05) { // 50ms gap is too large for 400Hz
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("deskewer"), steady_clock, 5000, 
            "Large gap (%.3f s) in IMU buffer. Slerp interpolation may be inaccurate.", msg_gap);
    }

    // Slerp Factor Calculation
    double dt = data_after.timestamp - data_before.timestamp;
    if (dt < 1e-9) return false; 
    
    double alpha = (timestamp - data_before.timestamp) / dt;

    // Perform Slerp on pre-processed quaternions
    orientation = data_before.orientation.slerp(static_cast<float>(alpha), data_after.orientation);
    
    return true;
}

} // namespace fs_perception