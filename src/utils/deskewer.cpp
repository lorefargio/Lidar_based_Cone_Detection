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

    double sweep_start_time = cloud->points.front().timestamp;
    double sweep_end_time = cloud->points.back().timestamp;
    
    if (sweep_start_time < 1.0) {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("deskewer"), steady_clock, 5000, 
            "Lidar points have invalid timestamps. Check driver config.");
        return false;
    }

    // 1. Snapshot of IMU buffer to avoid per-point locking
    std::vector<LidarOrientations> local_imu_buffer;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (imu_buffer_.size() < 2) return false;
        
        // Range check before copying everything
        if (sweep_start_time < imu_buffer_.front().timestamp || sweep_end_time > imu_buffer_.back().timestamp) {
            return false;
        }
        local_imu_buffer.assign(imu_buffer_.begin(), imu_buffer_.end());
    }

    size_t last_idx = 0;
    Eigen::Quaternionf start_orientation;
    if (!getInterpolatedOrientation(sweep_start_time, local_imu_buffer, last_idx, start_orientation)) {
        return false;
    }

    Eigen::Quaternionf start_orientation_inv = start_orientation.inverse();
    Eigen::Vector3f lever_arm = config_.static_imu_to_lidar;
    bool compensate_translation = config_.use_translation && lever_arm.norm() > 0.001f;

    // 2. Batch processing with monotonic search
    for (auto& point : cloud->points) {
        Eigen::Quaternionf current_orientation;
        // Search continues from last_idx due to chronological point ordering
        if (getInterpolatedOrientation(point.timestamp, local_imu_buffer, last_idx, current_orientation)) {
            
            // Optimized math: R_rel = start_inv * current
            Eigen::Quaternionf relative_rotation = start_orientation_inv * current_orientation;
            Eigen::Vector3f p(point.x, point.y, point.z);
            
            if (compensate_translation) {
                // p_corrected = R_rel * (p + lever) - lever
                Eigen::Vector3f p_corrected = relative_rotation * (p + lever_arm) - lever_arm;
                point.x = p_corrected.x();
                point.y = p_corrected.y();
                point.z = p_corrected.z();
            } else {
                // p_corrected = R_rel * p
                Eigen::Vector3f p_corrected = relative_rotation * p;
                point.x = p_corrected.x();
                point.y = p_corrected.y();
                point.z = p_corrected.z();
            }
        }
    }

    return true;
}

bool Deskewer::getInterpolatedOrientation(double timestamp, 
                                          const std::vector<LidarOrientations>& buffer,
                                          size_t& last_idx,
                                          Eigen::Quaternionf& orientation) {
    if (buffer.size() < 2) return false;

    // Linear monotonic search starting from last_idx
    size_t i = last_idx;
    while (i + 1 < buffer.size() && buffer[i + 1].timestamp < timestamp) {
        i++;
    }
    last_idx = i;

    // Ensure timestamp is within the found interval
    if (i + 1 >= buffer.size() || buffer[i].timestamp > timestamp || buffer[i+1].timestamp < timestamp) {
        // Fallback to binary search if monotonic search fails (e.g. non-ordered points)
        auto it = std::lower_bound(buffer.begin(), buffer.end(), timestamp,
            [](const LidarOrientations& data, double ts) {
                return data.timestamp < ts;
            });
        if (it == buffer.begin() || it == buffer.end()) return false;
        i = std::distance(buffer.begin(), it) - 1;
    }

    const auto& b = buffer[i];
    const auto& a = buffer[i+1];

    double dt = a.timestamp - b.timestamp;
    if (dt < 1e-9) return false; 
    
    double alpha = (timestamp - b.timestamp) / dt;
    orientation = b.orientation.slerp(static_cast<float>(alpha), a.orientation);
    
    return true;
}

} // namespace fs_perception