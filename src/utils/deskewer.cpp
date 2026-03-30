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

    const double sweep_start_time = cloud->points.front().timestamp;
    const double sweep_end_time = cloud->points.back().timestamp;
    const double sweep_duration = sweep_end_time - sweep_start_time;
    
    // Safety check for valid timestamps
    if (sweep_start_time < 1.0 || sweep_duration < 1e-6) return false;

    // 1. Snapshot of IMU buffer (Copy pointers/structs to local memory)
    std::vector<LidarOrientations> local_buffer;
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (imu_buffer_.size() < 2) return false;
        
        // Ensure the entire sweep is covered by the buffer
        if (sweep_start_time < imu_buffer_.front().timestamp || sweep_end_time > imu_buffer_.back().timestamp) {
            return false;
        }
        local_buffer.assign(imu_buffer_.begin(), imu_buffer_.end());
    }

    // 2. Pre-calculate Keyframes (O(K) where K << N points)
    // We sample the orientation every 1ms (approx 100 samples for a 10Hz sweep)
    const int num_keyframes = static_cast<int>(sweep_duration * 1000.0) + 2; 
    const double time_step = sweep_duration / (num_keyframes - 1);
    
    std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf>> keyframe_orientations(num_keyframes);
    size_t last_search_idx = 0;

    for (int i = 0; i < num_keyframes; ++i) {
        double t = sweep_start_time + i * time_step;
        getInterpolatedOrientation(t, local_buffer, last_search_idx, keyframe_orientations[i]);
    }

    const Eigen::Quaternionf start_orientation_inv = keyframe_orientations[0].inverse();
    const Eigen::Vector3f lever_arm = config_.static_imu_to_lidar;
    const bool use_translation = config_.use_translation && lever_arm.norm() > 0.001f;

    // 3. Massively Parallel Deskewing (O(N) with OpenMP and SIMD)
    const int n_points = static_cast<int>(cloud->points.size());

    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n_points; ++i) {
        auto& point = cloud->points[i];
        
        // Fast Index Mapping (O(1) instead of binary search)
        double relative_t = point.timestamp - sweep_start_time;
        int k_idx = static_cast<int>(relative_t / time_step);
        k_idx = std::clamp(k_idx, 0, num_keyframes - 2);
        
        float alpha = static_cast<float>((relative_t - k_idx * time_step) / time_step);
        alpha = std::clamp(alpha, 0.0f, 1.0f);

        // Fast NLERP (Interpolation of coefficients + Normalization)
        // Order of magnitude faster than SLERP and visually indistinguishable for small steps.
        Eigen::Quaternionf current_q;
        current_q.coeffs() = keyframe_orientations[k_idx].coeffs() * (1.0f - alpha) + 
                             keyframe_orientations[k_idx+1].coeffs() * alpha;
        current_q.normalize();

        // Transformation Application: p_corrected = R_relative * p
        // R_relative = q_start_inv * q_curr
        Eigen::Quaternionf relative_q = start_orientation_inv * current_q;
        Eigen::Vector3f p(point.x, point.y, point.z);

        if (use_translation) {
            // Simplified Lever Arm Correction: p_corrected = R_rel * (p + lever) - lever
            Eigen::Vector3f p_corrected = relative_q * (p + lever_arm) - lever_arm;
            point.x = p_corrected.x(); point.y = p_corrected.y(); point.z = p_corrected.z();
        } else {
            Eigen::Vector3f p_corrected = relative_q * p;
            point.x = p_corrected.x(); point.y = p_corrected.y(); point.z = p_corrected.z();
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