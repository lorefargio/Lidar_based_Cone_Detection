#include "utils/deskew_engine.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace fs_fusion {

pcl::PointCloud<pcl::PointXYZI>::Ptr DeskewEngine::deskewPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& raw_lidar,
    uint64_t target_ts_ns,
    bool do_deskew,
    const TransformManager& tf_manager,
    const ImuInterpolator& imu_interpolator,
    Eigen::Vector3d& out_accel) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*raw_lidar, *cloud);

    int ts_offset = -1;
    for (const auto& field : raw_lidar->fields) {
        if (field.name == "timestamp" || field.name == "time" || field.name == "t") {
            ts_offset = field.offset;
            break;
        }
    }

    std::string world_frame = tf_manager.getTfBuffer()->_frameExists("odom") ? "odom" : "map";
    Eigen::Vector3d v_world = const_cast<TransformManager&>(tf_manager).estimateLinearVelocity(target_ts_ns, world_frame, "zed_camera_link");
    Eigen::Matrix4d T_w_cl = const_cast<TransformManager&>(tf_manager).lookupTransformSafe(world_frame, "zed_camera_link", target_ts_ns);
    Eigen::Matrix3d R_w_cl = T_w_cl.block<3, 3>(0, 0);
    Eigen::Vector3d v_cl = R_w_cl.transpose() * v_world;

    Eigen::Matrix4d T_cl_l = const_cast<TransformManager&>(tf_manager).lookupTransformSafe("zed_camera_link", raw_lidar->header.frame_id, target_ts_ns);
    Eigen::Matrix3d R_cl_l = T_cl_l.block<3, 3>(0, 0);
    Eigen::Vector3d t_cl_l = T_cl_l.block<3, 1>(0, 3);

    out_accel = imu_interpolator.getInterpolatedLinearAccel(target_ts_ns);

    if (!do_deskew) return cloud;

    int points_num = cloud->points.size();
    uint8_t* raw_data_ptr = const_cast<uint8_t*>(&raw_lidar->data[0]);

    #pragma omp parallel for
    for (int i = 0; i < points_num; ++i) {
        auto& p = cloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        double pt_time_offset = 0.0;
        if (ts_offset >= 0) {
            float* f_t = reinterpret_cast<float*>(raw_data_ptr + i * raw_lidar->point_step + ts_offset);
            pt_time_offset = static_cast<double>(*f_t);
        }

        uint64_t pt_ts_ns = target_ts_ns + static_cast<uint64_t>(pt_time_offset * 1e9);
        double dt = (static_cast<double>(pt_ts_ns) - static_cast<double>(target_ts_ns)) / 1e9;

        Eigen::Vector3d p_l(p.x, p.y, p.z);
        Eigen::Vector3d p_cl = R_cl_l * p_l + t_cl_l;

        Eigen::Vector3d omega_lidar = imu_interpolator.getInterpolatedAngularVel(pt_ts_ns);
        Eigen::Vector3d omega_cl = omega_lidar;

        double theta = dt * omega_cl.norm();
        Eigen::Vector3d p_rotated = p_cl;
        if (theta > 1e-6) {
            Eigen::Vector3d axis = omega_cl.normalized();
            p_rotated = p_cl * std::cos(theta) + axis.cross(p_cl) * std::sin(theta) +
                        axis * axis.dot(p_cl) * (1.0 - std::cos(theta));
        }

        Eigen::Vector3d p_deskewed_cl = p_rotated + dt * v_cl;
        Eigen::Vector3d p_deskewed_l = R_cl_l.transpose() * (p_deskewed_cl - t_cl_l);

        p.x = p_deskewed_l.x();
        p.y = p_deskewed_l.y();
        p.z = p_deskewed_l.z();
    }

    return cloud;
}

} // namespace fs_fusion
