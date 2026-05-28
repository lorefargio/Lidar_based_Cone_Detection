#include "utils/imu_interpolator.hpp"
#include <algorithm>

namespace fs_fusion {

void ImuInterpolator::addImuData(const ImuData& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_history_.push_back(data);
    while (imu_history_.size() > 2000) {
        imu_history_.erase(imu_history_.begin());
    }
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
    double alpha = static_cast<double>(ts_ns - prev->ts_ns) / static_cast<double>(it->ts_ns - prev->ts_ns);
    return (1.0 - alpha) * prev->linear_accel + alpha * it->linear_accel;
}

} // namespace fs_fusion
