#ifndef LIDAR_PERCEPTION_CLOCK_ALIGNER_HPP_
#define LIDAR_PERCEPTION_CLOCK_ALIGNER_HPP_

#include <cstdint>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace fs_fusion {

/**
 * @class ClockAligner
 * @brief Manages clock synchronization and timestamp alignment between dynamic TF/system clocks and LiDAR streams.
 */
class ClockAligner {
public:
    ClockAligner() = default;
    ~ClockAligner() = default;

    /**
     * @brief Registers the first camera/system frame timestamp.
     * @param ts_ns Camera/system timestamp in nanoseconds.
     * @param logger ROS 2 logger for reporting status.
     */
    void updateCameraTime(uint64_t ts_ns, const rclcpp::Logger& logger);

    /**
     * @brief Registers the first LiDAR scan timestamp.
     * @param ts_ns LiDAR timestamp in nanoseconds.
     * @param logger ROS 2 logger for reporting status.
     */
    void updateLidarTime(uint64_t ts_ns, const rclcpp::Logger& logger);

    /**
     * @brief Dynamically updates and smooths the clock offset between LiDAR hardware time and PC system time.
     * @param raw_lidar_ts_ns LiDAR hardware timestamp in nanoseconds.
     * @param pc_recv_ts_ns PC system timestamp at message reception in nanoseconds.
     */
    void updateLidarTimeDynamic(uint64_t raw_lidar_ts_ns, uint64_t pc_recv_ts_ns);

    /**
     * @brief Checks if clock alignment calibration has completed.
     * @return True if initialized, false otherwise.
     */
    bool isInitialized() const;

    /**
     * @brief Maps a LiDAR timestamp into the dynamic TF clock domain.
     * @param ts_ns LiDAR raw timestamp in nanoseconds.
     * @return Aligned timestamp in system/TF clock domain.
     */
    uint64_t alignTimestamp(uint64_t ts_ns) const;

    /**
     * @brief Gets the calculated clock offset.
     * @return Offset in nanoseconds.
     */
    int64_t getOffset() const;

private:
    mutable std::mutex mutex_;
    uint64_t first_cam_ts_{0};
    uint64_t first_lidar_ts_{0};
    uint64_t latest_cam_ts_{0};
    int64_t initial_clock_offset_{0};
    int64_t running_offset_{0};
    bool clock_offset_initialized_{false};
};

} // namespace fs_fusion

#endif // LIDAR_PERCEPTION_CLOCK_ALIGNER_HPP_
