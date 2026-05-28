#ifndef LIDAR_CAMERA_FUSION_NODE_TRANSFORM_MANAGER_HPP_
#define LIDAR_CAMERA_FUSION_NODE_TRANSFORM_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>

namespace fs_fusion {

/**
 * @class TransformManager
 * @brief Manages spatial transformations, sensor extrinsics correction, and linear velocity estimation.
 */
class TransformManager {
public:
    /**
     * @brief Constructor that initializes calibration parameters and TF2 listeners.
     */
    TransformManager(const rclcpp::Clock::SharedPtr& clock,
                     const std::vector<double>& rot,
                     const std::vector<double>& trans,
                     double roll_adj, double pitch_adj, double yaw_adj);

    ~TransformManager() = default;

    /**
     * @brief Computes rotation matrix from roll, pitch, and yaw Euler angles.
     */
    Eigen::Matrix3d getEulerRotationMatrix(double roll_deg, double pitch_deg, double yaw_deg);

    /**
     * @brief Injects standard coordinate frames into TF2 buffer as dynamic fallbacks.
     */
    void injectStaticTFs(const rclcpp::Time& now, const std::string& lidar_frame_id = "hesai_40");

    /**
     * @brief Thread-safe transform lookup with zero-latency fallbacks.
     */
    Eigen::Matrix4d lookupTransformSafe(const std::string& target, const std::string& source, uint64_t ts_ns);

    /**
     * @brief Performs a 5-point symmetric local polynomial sliding window linear regression to estimate linear velocity.
     * @param ts_ns Target timestamp in nanoseconds.
     * @param world_frame Name of target coordinate frame.
     * @param body_frame Name of source coordinate frame.
     * @return Estimated linear velocity vector.
     */
    Eigen::Vector3d estimateLinearVelocity(uint64_t ts_ns, const std::string& world_frame, const std::string& body_frame);

    const Eigen::Matrix4d& getLidarToCameraOpt() const;
    const Eigen::Matrix3d& getRotExt() const;
    const Eigen::Vector3d& getTransExt() const;
    const std::shared_ptr<tf2_ros::Buffer>& getTfBuffer() const;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    Eigen::Matrix3d R_ext_;
    Eigen::Vector3d t_ext_;
    Eigen::Matrix4d T_lidar_to_camera_opt_;
};

} // namespace fs_fusion

#endif // LIDAR_CAMERA_FUSION_NODE_TRANSFORM_MANAGER_HPP_
