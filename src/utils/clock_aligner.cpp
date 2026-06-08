#include "utils/clock_aligner.hpp"

namespace fs_fusion {

void ClockAligner::updateCameraTime(uint64_t ts_ns, const rclcpp::Logger& logger) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cam_ts_ = ts_ns;
    if (clock_offset_initialized_) return;
    if (first_cam_ts_ == 0) {
        first_cam_ts_ = ts_ns;
    }
    if (first_lidar_ts_ > 0) {
        initial_clock_offset_ = static_cast<int64_t>(first_lidar_ts_) - static_cast<int64_t>(first_cam_ts_);
        if (std::abs(initial_clock_offset_) < 10000000000LL) { // < 10s
            initial_clock_offset_ = 0;
            RCLCPP_INFO(logger, "Automatic Clock Alignment: Sensors are already in the same clock domain. Offset is 0.");
        } else {
            RCLCPP_INFO(logger, "Automatic Clock Alignment: Offset is %.3f seconds.", initial_clock_offset_ / 1e9);
        }
        running_offset_ = initial_clock_offset_;
        clock_offset_initialized_ = true;
    }
}

void ClockAligner::updateLidarTime(uint64_t ts_ns, const rclcpp::Logger& logger) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (clock_offset_initialized_) return;
    if (first_lidar_ts_ == 0) {
        first_lidar_ts_ = ts_ns;
    }
    if (first_cam_ts_ > 0) {
        initial_clock_offset_ = static_cast<int64_t>(first_lidar_ts_) - static_cast<int64_t>(first_cam_ts_);
        if (std::abs(initial_clock_offset_) < 10000000000LL) { // < 10s
            initial_clock_offset_ = 0;
            RCLCPP_INFO(logger, "Automatic Clock Alignment: Sensors are already in the same clock domain. Offset is 0.");
        } else {
            RCLCPP_INFO(logger, "Automatic Clock Alignment: Offset is %.3f seconds.", initial_clock_offset_ / 1e9);
        }
        running_offset_ = initial_clock_offset_;
        clock_offset_initialized_ = true;
    }
}

void ClockAligner::updateLidarTimeDynamic(uint64_t raw_lidar_ts_ns, uint64_t pc_recv_ts_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Primary sync uses latest camera IMU header stamp to stay robust during rosbag play.
    // Falls back to PC arrival time if camera frames have not started publishing.
    uint64_t ref_ts_ns = (latest_cam_ts_ > 0) ? latest_cam_ts_ : pc_recv_ts_ns;
    
    int64_t current_offset = static_cast<int64_t>(raw_lidar_ts_ns) - static_cast<int64_t>(ref_ts_ns);
    
    if (!clock_offset_initialized_) {
        if (std::abs(current_offset) < 10000000000LL) { // < 10s
            initial_clock_offset_ = 0;
            running_offset_ = 0;
        } else {
            initial_clock_offset_ = current_offset;
            running_offset_ = current_offset;
        }
        clock_offset_initialized_ = true;
    } else {
        if (initial_clock_offset_ != 0) {
            // Continuous clock drift and network transmission latency tracking.
            const double alpha = 0.005;
            running_offset_ = static_cast<int64_t>((1.0 - alpha) * static_cast<double>(running_offset_) + alpha * static_cast<double>(current_offset));
        }
    }
}

bool ClockAligner::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clock_offset_initialized_;
}

uint64_t ClockAligner::alignTimestamp(uint64_t ts_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (clock_offset_initialized_) {
        return static_cast<uint64_t>(static_cast<int64_t>(ts_ns) - running_offset_);
    }
    return ts_ns;
}

int64_t ClockAligner::getOffset() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return running_offset_;
}

} // namespace fs_fusion
