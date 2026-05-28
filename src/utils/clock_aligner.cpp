#include "utils/clock_aligner.hpp"

namespace fs_fusion {

void ClockAligner::updateCameraTime(uint64_t ts_ns, const rclcpp::Logger& logger) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (clock_offset_initialized_) return;
    if (first_cam_ts_ == 0) {
        first_cam_ts_ = ts_ns;
    }
    if (first_lidar_ts_ > 0) {
        initial_clock_offset_ = static_cast<int64_t>(first_lidar_ts_) - static_cast<int64_t>(first_cam_ts_);
        clock_offset_initialized_ = true;
        RCLCPP_INFO(logger, "Automatic Clock Alignment: Offset is %.3f seconds.", initial_clock_offset_ / 1e9);
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
        clock_offset_initialized_ = true;
        RCLCPP_INFO(logger, "Automatic Clock Alignment: Offset is %.3f seconds.", initial_clock_offset_ / 1e9);
    }
}

bool ClockAligner::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clock_offset_initialized_;
}

uint64_t ClockAligner::alignTimestamp(uint64_t ts_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (clock_offset_initialized_) {
        return static_cast<uint64_t>(static_cast<int64_t>(ts_ns) - initial_clock_offset_);
    }
    return ts_ns;
}

int64_t ClockAligner::getOffset() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initial_clock_offset_;
}

} // namespace fs_fusion
