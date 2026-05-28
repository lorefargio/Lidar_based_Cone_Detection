#include "utils/transform_manager.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>

namespace fs_fusion {

TransformManager::TransformManager(const rclcpp::Clock::SharedPtr& clock,
                                   const std::vector<double>& rot,
                                   const std::vector<double>& trans,
                                   double roll_adj, double pitch_adj, double yaw_adj) {
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    Eigen::Matrix3d R_ext_base;
    if (rot.size() == 9) {
        for (int i = 0; i < 9; ++i) R_ext_base(i / 3, i % 3) = rot[i];
    } else {
        R_ext_base = Eigen::Matrix3d::Identity();
    }
    t_ext_ = (trans.size() == 3) ? Eigen::Vector3d(trans[0], trans[1], trans[2]) : Eigen::Vector3d::Zero();

    Eigen::Matrix3d R_correction = getEulerRotationMatrix(roll_adj, pitch_adj, yaw_adj);
    R_ext_ = R_ext_base * R_correction;

    T_lidar_to_camera_opt_ = Eigen::Matrix4d::Identity();
    T_lidar_to_camera_opt_.block<3, 3>(0, 0) = R_ext_;
    T_lidar_to_camera_opt_.block<3, 1>(0, 3) = t_ext_;
}

Eigen::Matrix3d TransformManager::getEulerRotationMatrix(double roll_deg, double pitch_deg, double yaw_deg) {
    double r = roll_deg * M_PI / 180.0;
    double p = pitch_deg * M_PI / 180.0;
    double y = yaw_deg * M_PI / 180.0;

    Eigen::AngleAxisd roll(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(y, Eigen::Vector3d::UnitZ());

    return (yaw * pitch * roll).matrix();
}

void TransformManager::injectStaticTFs(const rclcpp::Time& now, const std::string& lidar_frame_id) {
    geometry_msgs::msg::TransformStamped t_ext_msg;
    t_ext_msg.header.stamp = now;
    t_ext_msg.header.frame_id = "zed_left_camera_frame_optical";
    t_ext_msg.child_frame_id = lidar_frame_id;

    Eigen::Quaterniond q(R_ext_);
    t_ext_msg.transform.translation.x = t_ext_.x();
    t_ext_msg.transform.translation.y = t_ext_.y();
    t_ext_msg.transform.translation.z = t_ext_.z();
    t_ext_msg.transform.rotation.x = q.x();
    t_ext_msg.transform.rotation.y = q.y();
    t_ext_msg.transform.rotation.z = q.z();
    t_ext_msg.transform.rotation.w = q.w();

    tf_buffer_->setTransform(t_ext_msg, "fs_fusion_static", true);

    geometry_msgs::msg::TransformStamped t_cam1;
    t_cam1.header.stamp = now;
    t_cam1.header.frame_id = "zed_camera_link";
    t_cam1.child_frame_id = "zed_camera_center";
    t_cam1.transform.translation.z = 0.015;
    t_cam1.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(t_cam1, "fs_fusion_static", true);

    geometry_msgs::msg::TransformStamped t_cam2;
    t_cam2.header.stamp = now;
    t_cam2.header.frame_id = "zed_camera_center";
    t_cam2.child_frame_id = "zed_left_camera_frame";
    t_cam2.transform.translation.x = -0.01;
    t_cam2.transform.translation.y = 0.05995845;
    t_cam2.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(t_cam2, "fs_fusion_static", true);

    geometry_msgs::msg::TransformStamped t_cam3;
    t_cam3.header.stamp = now;
    t_cam3.header.frame_id = "zed_left_camera_frame";
    t_cam3.child_frame_id = "zed_left_camera_frame_optical";
    t_cam3.transform.translation.x = 0.0;
    t_cam3.transform.translation.y = 0.0;
    t_cam3.transform.translation.z = 0.0;
    t_cam3.transform.rotation.x = 0.5;
    t_cam3.transform.rotation.y = -0.5;
    t_cam3.transform.rotation.z = 0.5;
    t_cam3.transform.rotation.w = -0.5;
    tf_buffer_->setTransform(t_cam3, "fs_fusion_static", true);
}

Eigen::Matrix4d TransformManager::lookupTransformSafe(const std::string& target, const std::string& source, uint64_t ts_ns) {
    try {
        rclcpp::Time rcl_time(static_cast<int64_t>(ts_ns));
        geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(target, source, rcl_time);
        return tf2::transformToEigen(tf_msg.transform).matrix();
    } catch (const tf2::TransformException& ex) {
        try {
            geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
            return tf2::transformToEigen(tf_msg.transform).matrix();
        } catch (const tf2::TransformException& ex2) {
            return Eigen::Matrix4d::Identity();
        }
    }
}

Eigen::Vector3d TransformManager::estimateLinearVelocity(uint64_t ts_ns, const std::string& world_frame, const std::string& body_frame) {
    uint64_t dt_ns = 25000000ULL;
    std::vector<double> u_sec = {-0.05, -0.025, 0.0, 0.025, 0.05};
    std::vector<Eigen::Vector3d> p(5);

    bool all_ok = true;
    for (int i = 0; i < 5; ++i) {
        int64_t t_i = static_cast<int64_t>(ts_ns) + (i - 2) * static_cast<int64_t>(dt_ns);
        Eigen::Matrix4d T = lookupTransformSafe(world_frame, body_frame, t_i);
        if (T == Eigen::Matrix4d::Identity()) {
            all_ok = false;
            break;
        }
        p[i] = T.block<3, 1>(0, 3);
    }

    if (all_ok) {
        Eigen::Vector3d slope = Eigen::Vector3d::Zero();
        for (int i = 0; i < 5; ++i) {
            slope += u_sec[i] * p[i];
        }
        return slope / 0.00625;
    }

    Eigen::Matrix4d T0 = lookupTransformSafe(world_frame, body_frame, ts_ns - 20000000ULL);
    Eigen::Matrix4d T1 = lookupTransformSafe(world_frame, body_frame, ts_ns);
    if (T0 != Eigen::Matrix4d::Identity() && T1 != Eigen::Matrix4d::Identity()) {
        return (T1.block<3, 1>(0, 3) - T0.block<3, 1>(0, 3)) / 0.02;
    }

    return Eigen::Vector3d::Zero();
}

const Eigen::Matrix4d& TransformManager::getLidarToCameraOpt() const { return T_lidar_to_camera_opt_; }
const Eigen::Matrix3d& TransformManager::getRotExt() const { return R_ext_; }
const Eigen::Vector3d& TransformManager::getTransExt() const { return t_ext_; }
const std::shared_ptr<tf2_ros::Buffer>& TransformManager::getTfBuffer() const { return tf_buffer_; }

} // namespace fs_fusion
