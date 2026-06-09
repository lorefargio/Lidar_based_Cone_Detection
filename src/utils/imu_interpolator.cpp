#include "utils/imu_interpolator.hpp"
#include <algorithm>
#include <cmath>

namespace fs_fusion {

void ImuInterpolator::addImuData(const ImuData& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    ImuData filtered_data = data;
    
    // Clamp extreme/limit/unstable IMU readings to physically realistic boundaries
    const double MAX_ANGULAR_VEL = 15.0;  // rad/s (approx 860 deg/s)
    const double MAX_LINEAR_ACCEL = 80.0;  // m/s^2 (approx 8G)
    
    filtered_data.angular_vel = data.angular_vel.unaryExpr([MAX_ANGULAR_VEL](double v) {
        return std::isnan(v) ? 0.0 : std::clamp(v, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
    });
    filtered_data.linear_accel = data.linear_accel.unaryExpr([MAX_LINEAR_ACCEL](double v) {
        return std::isnan(v) ? 0.0 : std::clamp(v, -MAX_LINEAR_ACCEL, MAX_LINEAR_ACCEL);
    });

    // Maintain strict chronological sorting in cache (guards std::lower_bound in interpolation)
    auto insert_it = std::upper_bound(imu_history_.begin(), imu_history_.end(), filtered_data.ts_ns,
        [](uint64_t t, const ImuData& a) { return t < a.ts_ns; });
    imu_history_.insert(insert_it, filtered_data);
    
    while (imu_history_.size() > 2000) {
        imu_history_.erase(imu_history_.begin());
    }
}

std::vector<ImuData> ImuInterpolator::getImuDataInWindow(uint64_t start_ts_ns, uint64_t end_ts_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ImuData> result;
    if (imu_history_.empty()) return result;

    auto it_start = std::lower_bound(imu_history_.begin(), imu_history_.end(), start_ts_ns,
        [](const ImuData& a, uint64_t t) { return a.ts_ns < t; });

    auto it_end = std::upper_bound(imu_history_.begin(), imu_history_.end(), end_ts_ns,
        [](uint64_t t, const ImuData& a) { return t < a.ts_ns; });

    result.assign(it_start, it_end);
    return result;
}

Eigen::Vector3d ImuInterpolator::getInterpolatedAngularVel(uint64_t ts_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_history_.empty()) return Eigen::Vector3d::Zero();
    if (ts_ns <= imu_history_.front().ts_ns) return imu_history_.front().angular_vel;
    if (ts_ns >= imu_history_.back().ts_ns) return imu_history_.back().angular_vel;

    auto it = std::lower_bound(imu_history_.begin(), imu_history_.end(), ts_ns,
        [](const ImuData& a, uint64_t t) { return a.ts_ns < t; });

    if (it == imu_history_.begin()) return it->angular_vel;
    auto prev = it - 1;
    
    // Safety guard to prevent uint64_t subtraction underflow
    if (ts_ns <= prev->ts_ns) return prev->angular_vel;
    if (ts_ns >= it->ts_ns) return it->angular_vel;
    
    double alpha = static_cast<double>(ts_ns - prev->ts_ns) / static_cast<double>(it->ts_ns - prev->ts_ns);
    return (1.0 - alpha) * prev->angular_vel + alpha * it->angular_vel;
}

Eigen::Vector3d ImuInterpolator::getInterpolatedLinearAccel(uint64_t ts_ns) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (imu_history_.empty()) return Eigen::Vector3d::Zero();
    if (ts_ns <= imu_history_.front().ts_ns) return imu_history_.front().linear_accel;
    if (ts_ns >= imu_history_.back().ts_ns) return imu_history_.back().linear_accel;

    auto it = std::lower_bound(imu_history_.begin(), imu_history_.end(), ts_ns,
        [](const ImuData& a, uint64_t t) { return a.ts_ns < t; });

    if (it == imu_history_.begin()) return it->linear_accel;
    auto prev = it - 1;
    
    // Safety guard to prevent uint64_t subtraction underflow
    if (ts_ns <= prev->ts_ns) return prev->linear_accel;
    if (ts_ns >= it->ts_ns) return it->linear_accel;
    
    double alpha = static_cast<double>(ts_ns - prev->ts_ns) / static_cast<double>(it->ts_ns - prev->ts_ns);
    return (1.0 - alpha) * prev->linear_accel + alpha * it->linear_accel;
}

} // namespace fs_fusion
