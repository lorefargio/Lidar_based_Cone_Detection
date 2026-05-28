#ifndef LIDAR_CAMERA_FUSION_NODE_DESKEW_ENGINE_HPP_
#define LIDAR_CAMERA_FUSION_NODE_DESKEW_ENGINE_HPP_

#include "transform_manager.hpp"
#include "imu_interpolator.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs_fusion {

/**
 * @class DeskewEngine
 * @brief Compensates for motion distortion (rolling-shutter effect) in raw LiDAR scans using high-frequency IMU integration.
 */
class DeskewEngine {
public:
    DeskewEngine() = default;
    ~DeskewEngine() = default;

    /**
     * @brief Computes motion-compensated (deskewed) point cloud.
     * @param raw_lidar ROS PointCloud2 sensor payload.
     * @param target_ts_ns Aligned reference timestamp.
     * @param do_deskew Flags whether motion correction should run.
     * @param tf_manager Const reference to the TransformManager.
     * @param imu_interpolator Const reference to the ImuInterpolator.
     * @param out_accel Output parameter returning interpolated IMU linear acceleration.
     * @return Motion-corrected point cloud.
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr deskewPointCloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& raw_lidar,
        uint64_t target_ts_ns,
        bool do_deskew,
        const TransformManager& tf_manager,
        const ImuInterpolator& imu_interpolator,
        Eigen::Vector3d& out_accel);
};

} // namespace fs_fusion

#endif // LIDAR_CAMERA_FUSION_NODE_DESKEW_ENGINE_HPP_
