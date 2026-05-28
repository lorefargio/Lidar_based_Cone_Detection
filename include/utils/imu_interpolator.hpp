#ifndef LIDAR_CAMERA_FUSION_NODE_IMU_INTERPOLATOR_HPP_
#define LIDAR_CAMERA_FUSION_NODE_IMU_INTERPOLATOR_HPP_

#include <Eigen/Dense>
#include <vector>
#include <mutex>
#include <cstdint>

namespace fs_fusion {

/**
 * @struct ImuData
 * @brief Thread-safe telemetry payload from a high-frequency inertial sensor.
 */
struct ImuData {
    uint64_t ts_ns;              ///< Sensor reading timestamp in nanoseconds
    Eigen::Vector3d angular_vel;  ///< Angular velocity vector (rad/s) in the local frame
    Eigen::Vector3d linear_accel; ///< Linear acceleration vector (m/s^2) in the local frame
};

/**
 * @class ImuInterpolator
 * @brief Manages thread-safe buffering and high-frequency interpolation of IMU telemetry.
 */
class ImuInterpolator {
public:
    ImuInterpolator() = default;
    ~ImuInterpolator() = default;

    /**
     * @brief Pushes new high-frequency IMU telemetry into history cache.
     * @param data ImuData payload.
     */
    void addImuData(const ImuData& data);

    /**
     * @brief Linearly interpolates vehicle angular velocity vector at target timestamp.
     * @param ts_ns Target timestamp in nanoseconds.
     * @return Interpolated 3D angular velocity vector.
     */
    Eigen::Vector3d getInterpolatedAngularVel(uint64_t ts_ns) const;

    /**
     * @brief Linearly interpolates vehicle linear acceleration vector at target timestamp.
     * @param ts_ns Target timestamp in nanoseconds.
     * @return Interpolated 3D linear acceleration vector.
     */
    Eigen::Vector3d getInterpolatedLinearAccel(uint64_t ts_ns) const;

private:
    mutable std::mutex mutex_;
    std::vector<ImuData> imu_history_;
};

} // namespace fs_fusion

#endif // LIDAR_CAMERA_FUSION_NODE_IMU_INTERPOLATOR_HPP_
