#include "node/perception_pipeline.hpp"
#include "filtering/ground_remover_factory.hpp"
#include "clustering/clusterer_factory.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
#include <Eigen/Geometry>

namespace lidar_perception {

PerceptionPipeline::PerceptionPipeline() : g_world_(0.0, -9.80665, 0.0) {}

PerceptionPipeline::~PerceptionPipeline() {}

void PerceptionPipeline::initialize(const PipelineConfig& config, const rclcpp::Clock::SharedPtr& clock) {
    // 1. World Gravity Setup
    if (config.world_up_axis == "y") {
        g_world_ = Eigen::Vector3d(0.0, -9.80665, 0.0);
    } else if (config.world_up_axis == "z") {
        g_world_ = Eigen::Vector3d(0.0, 0.0, -9.80665);
    } else {
        throw std::runtime_error("Invalid world_up_axis value: '" + config.world_up_axis + "'. Must be 'y' or 'z'.");
    }

    // 2. Instantiate Ground Remover & Clusterer using Factories
    ground_remover_ = GroundRemoverFactory::create(config);
    clusterer_ = ClustererFactory::create(config);

    // 3. Instantiate Cone Estimator
    ConeEstimator::Config est_cfg;
    est_cfg.min_height = static_cast<float>(config.rule_min_height);
    est_cfg.max_height = static_cast<float>(config.rule_max_height);
    est_cfg.base_min_width = static_cast<float>(config.rule_base_min_width);
    est_cfg.max_width = static_cast<float>(config.rule_max_width);
    est_cfg.dynamic_width_decay = static_cast<float>(config.rule_dynamic_width_decay);
    est_cfg.min_points_at_10m = config.rule_min_points_at_10m;
    est_cfg.min_points_cap = config.rule_min_points_cap;
    est_cfg.min_intensity = static_cast<float>(config.rule_min_intensity);
    est_cfg.max_linearity = static_cast<float>(config.pca_max_linearity);
    est_cfg.max_planarity = static_cast<float>(config.pca_max_planarity);
    est_cfg.min_scatter = static_cast<float>(config.pca_min_scatter);
    est_cfg.min_verticality = static_cast<float>(config.pca_min_verticality);
    est_cfg.ground_z_level = static_cast<float>(config.sensor_z);

    estimator_ = std::make_unique<ConeEstimator>(est_cfg);

    // 4. Deskewing support components
    if (config.use_deskewing) {
        tf_manager_ = std::make_unique<fs_fusion::TransformManager>(
            clock, config.extrinsic_rotation, config.extrinsic_translation, 
            config.roll_deg, config.pitch_deg, config.yaw_deg);
            
        imu_interpolator_ = std::make_unique<fs_fusion::ImuInterpolator>();
    }
}

std::vector<Cone> PerceptionPipeline::run(
    PointCloudPtr& input_cloud, 
    uint64_t target_ts_ns, 
    const std::string& frame_id, 
    int frame_counter, 
    const PipelineConfig& config, 
    ClusterLogger* cluster_logger, 
    PerformanceProfiler* profiler,
    PointCloudPtr& obstacles_out,
    PointCloudPtr& ground_out,
    std::vector<PointCloudPtr>& merged_clusters_out) {

    std::vector<Cone> final_cones;
    if (input_cloud->empty()) return final_cones;

    // 1. Spatial Truncation (Pre-processing)
    preProcess(input_cloud, config);
    if (input_cloud->empty()) return final_cones;

    // 2. Motion Compensation (Deskewing)
    performDeskewing(target_ts_ns, frame_id, input_cloud, config, profiler);

    // 3. Ground Removal
    if (!ground_remover_) return final_cones;
    if (profiler) profiler->startTimer("ground_removal");
    ground_remover_->removeGround(input_cloud, obstacles_out, ground_out);
    if (profiler) profiler->stopTimer("ground_removal");

    // 4. Clustering & Merging
    performClusteringAndMerging(obstacles_out, merged_clusters_out, config, profiler);

    // 5. Cone geometric classification
    std::vector<Cone> candidate_cones;
    if (profiler) profiler->startTimer("estimation");
    for (size_t i = 0; i < merged_clusters_out.size(); ++i) {
        auto cone = estimator_->estimate(merged_clusters_out[i]);
        
        if (cluster_logger) {
            cone.features.frame_id = frame_counter;
            cone.features.cluster_id = static_cast<int>(i);
            cluster_logger->addCluster(cone.features);
        }

        if (cone.confidence > 0.5f) {
            candidate_cones.push_back(cone);
        }
    }
    if (profiler) profiler->stopTimer("estimation");

    // 6. Duplicate Suppression
    aggregateDuplicates(candidate_cones, final_cones, config, profiler);

    return final_cones;
}

void PerceptionPipeline::preProcess(const PointCloudPtr& cloud, const PipelineConfig& config) {
    if (cloud->empty()) return;

    const float max_range = static_cast<float>(config.max_range);
    const float max_r2 = max_range * max_range;
    const float min_r2 = 0.25f * 0.25f; // Internal sensor blind spot

    auto it = std::remove_if(cloud->points.begin(), cloud->points.end(), [&](const PointT& pt) {
        float r2 = pt.x * pt.x + pt.y * pt.y;
        return r2 > max_r2 || r2 < min_r2 || !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z);
    });

    if (it != cloud->points.end()) {
        cloud->points.erase(it, cloud->points.end());
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
}

void PerceptionPipeline::performDeskewing(
    uint64_t target_ts_ns, 
    const std::string& frame_id, 
    PointCloudPtr& processing_cloud, 
    const PipelineConfig& config, 
    PerformanceProfiler* profiler) {

    if (config.use_deskewing && tf_manager_ && imu_interpolator_) {
        if (imu_interpolator_->getCacheSize() == 0) return;

        if (profiler) profiler->startTimer("deskewing");

        // 1. Inject static transforms dynamically
        tf_manager_->injectStaticTFs(rclcpp::Time(static_cast<int64_t>(target_ts_ns)), frame_id);

        // 2. Lookup coordinate transforms and estimate LiDAR frame linear velocity
        std::string world_frame = tf_manager_->getTfBuffer()->_frameExists("odom") ? "odom" : "map";
        Eigen::Vector3d v_world = tf_manager_->estimateLinearVelocity(target_ts_ns, world_frame, frame_id);
        Eigen::Matrix4d T_w_l = tf_manager_->lookupTransformSafe(world_frame, frame_id, target_ts_ns);
        Eigen::Matrix3d R_w_l = T_w_l.block<3, 3>(0, 0);
        Eigen::Vector3d v_l = R_w_l.transpose() * v_world;

        // Lookup lever arm translation from TF buffer (origin of IMU/camera in LiDAR frame)
        Eigen::Matrix4d T_l_im = tf_manager_->lookupTransformSafe(frame_id, config.imu_frame, target_ts_ns);
        Eigen::Vector3d L_l = T_l_im.block<3, 1>(0, 3);

        // Precompute camera-to-LiDAR static rotation matrix based on axis mapping:
        // x_L = x_C, y_L = z_C, z_L = -y_C
        Eigen::Matrix3d R_l_c;
        R_l_c << 1.0,  0.0, 0.0,
                 0.0,  0.0, 1.0,
                 0.0, -1.0, 0.0;

        // Precompute gravity vector in LiDAR local frame
        Eigen::Vector3d g_l = R_w_l.transpose() * g_world_;

        // 3. Pre-integrate IMU trajectory
        // Get all IMU samples in a 300ms window centered on target_ts_ns
        std::vector<fs_fusion::ImuData> imu_samples = imu_interpolator_->getImuDataInWindow(
            target_ts_ns - 150000000ULL, // -150ms
            target_ts_ns + 150000000ULL  // +150ms
        );

        struct PoseState {
            uint64_t ts_ns;
            Eigen::Quaterniond q; // rotation relative to target
            Eigen::Vector3d p;    // translation relative to target
        };

        std::vector<PoseState> trajectory;

        if (imu_samples.size() >= 2) {
            int N = imu_samples.size();
            std::vector<Eigen::Quaterniond> rel_q(N);
            std::vector<Eigen::Vector3d> rel_x(N);
            std::vector<Eigen::Vector3d> rel_v(N);

            // Find closest sample index to target_ts_ns
            int m = 0;
            uint64_t min_dt = 999999999999ULL;
            for (int k = 0; k < N; ++k) {
                uint64_t diff = (imu_samples[k].ts_ns > target_ts_ns) ? 
                                (imu_samples[k].ts_ns - target_ts_ns) : 
                                (target_ts_ns - imu_samples[k].ts_ns);
                if (diff < min_dt) {
                    min_dt = diff;
                    m = k;
                }
            }

            // Initialize at closest index m (corresponding to target)
            rel_q[m] = Eigen::Quaterniond::Identity();
            rel_x[m] = Eigen::Vector3d::Zero();
            rel_v[m] = v_l;

            // Integrate Forward: k from m to N - 1
            for (int k = m; k < N - 1; ++k) {
                double dt = (static_cast<double>(imu_samples[k+1].ts_ns) - static_cast<double>(imu_samples[k].ts_ns)) / 1e9;
                if (dt <= 0.0 || dt > 0.1) dt = 0.01; // fallback to 100Hz delta

                Eigen::Vector3d omega_mean_cam = 0.5 * (imu_samples[k].angular_vel + imu_samples[k+1].angular_vel);
                Eigen::Vector3d accel_mean_cam = 0.5 * (imu_samples[k].linear_accel + imu_samples[k+1].linear_accel);

                Eigen::Vector3d omega_l = R_l_c * omega_mean_cam;
                Eigen::Vector3d accel_proper_l = R_l_c * accel_mean_cam;

                // Propagate rotation: rel_q[k+1] = rel_q[k] * dq
                Eigen::Vector3d rot_vec = dt * omega_l;
                Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
                double angle = rot_vec.norm();
                if (angle > 1e-8) {
                    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot_vec.normalized()));
                }
                rel_q[k+1] = rel_q[k] * dq;
                rel_q[k+1].normalize();

                // Compute centripetal acceleration in LiDAR frame at t_k
                Eigen::Vector3d centripetal_l = omega_l.cross(omega_l.cross(L_l));
                Eigen::Vector3d total_accel_proper_l = accel_proper_l + centripetal_l;

                // Rotate proper acceleration to target LiDAR coordinate frame (at t_0)
                Eigen::Vector3d accel_proper_target = rel_q[k] * total_accel_proper_l;
                Eigen::Vector3d accel_kinematic_target = accel_proper_target - g_l;

                rel_x[k+1] = rel_x[k] + rel_v[k] * dt + 0.5 * accel_kinematic_target * dt * dt;
                rel_v[k+1] = rel_v[k] + accel_kinematic_target * dt;
            }

            // Integrate Backward: k from m down to 1
            for (int k = m; k > 0; --k) {
                double dt = (static_cast<double>(imu_samples[k-1].ts_ns) - static_cast<double>(imu_samples[k].ts_ns)) / 1e9; // dt is negative
                if (std::abs(dt) > 0.1) dt = -0.01;

                Eigen::Vector3d omega_mean_cam = 0.5 * (imu_samples[k].angular_vel + imu_samples[k-1].angular_vel);
                Eigen::Vector3d accel_mean_cam = 0.5 * (imu_samples[k].linear_accel + imu_samples[k-1].linear_accel);

                Eigen::Vector3d omega_l = R_l_c * omega_mean_cam;
                Eigen::Vector3d accel_proper_l = R_l_c * accel_mean_cam;

                // Propagate rotation: rel_q[k-1] = rel_q[k] * dq
                Eigen::Vector3d rot_vec = dt * omega_l;
                Eigen::Quaterniond dq = Eigen::Quaterniond::Identity();
                double angle = rot_vec.norm();
                if (angle > 1e-8) {
                    dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rot_vec.normalized()));
                }
                rel_q[k-1] = rel_q[k] * dq;
                rel_q[k-1].normalize();

                // Compute centripetal acceleration in LiDAR frame at t_k
                Eigen::Vector3d centripetal_l = omega_l.cross(omega_l.cross(L_l));
                Eigen::Vector3d total_accel_proper_l = accel_proper_l + centripetal_l;

                // Rotate proper acceleration to target LiDAR coordinate frame (at t_0)
                Eigen::Vector3d accel_proper_target = rel_q[k] * total_accel_proper_l;
                Eigen::Vector3d accel_kinematic_target = accel_proper_target - g_l;

                // Integrate backward (dt is negative)
                rel_x[k-1] = rel_x[k] + rel_v[k] * dt + 0.5 * accel_kinematic_target * dt * dt;
                rel_v[k-1] = rel_v[k] + accel_kinematic_target * dt;
            }

            // Fill trajectory
            trajectory.resize(N);
            for (int k = 0; k < N; ++k) {
                trajectory[k].ts_ns = imu_samples[k].ts_ns;
                trajectory[k].q = rel_q[k];
                trajectory[k].p = rel_x[k];
            }
        }

        bool trajectory_ok = (trajectory.size() >= 2);

        int points_num = processing_cloud->points.size();
        double total_displacement = 0.0;
        double max_displacement = 0.0;
        int valid_points_count = 0;

        #pragma omp parallel for schedule(static) reduction(+:total_displacement, valid_points_count) reduction(max:max_displacement)
        for (int i = 0; i < points_num; ++i) {
            auto& p = processing_cloud->points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

            uint64_t pt_ts_ns = static_cast<uint64_t>(p.timestamp * 1e9);
            double dt = (static_cast<double>(pt_ts_ns) - static_cast<double>(target_ts_ns)) / 1e9;

            if (std::abs(dt) > 0.15) continue;

            Eigen::Vector3d p_l(p.x, p.y, p.z);
            Eigen::Vector3d p_deskewed_l;

            if (trajectory_ok) {
                // Find adjacent poses in pre-integrated trajectory
                auto it = std::lower_bound(trajectory.begin(), trajectory.end(), pt_ts_ns,
                    [](const PoseState& s, uint64_t t) { return s.ts_ns < t; });

                if (it == trajectory.begin()) {
                    Eigen::Vector3d p_lever = p_l - L_l;
                    p_deskewed_l = trajectory.front().q * p_lever + L_l + trajectory.front().p;
                } else if (it == trajectory.end()) {
                    Eigen::Vector3d p_lever = p_l - L_l;
                    p_deskewed_l = trajectory.back().q * p_lever + L_l + trajectory.back().p;
                } else {
                    const auto& s_prev = *(it - 1);
                    const auto& s_next = *it;
                    double dt_seg = static_cast<double>(s_next.ts_ns - s_prev.ts_ns) / 1e9;
                    double alpha = 0.0;
                    if (dt_seg > 1e-8) {
                        alpha = static_cast<double>(pt_ts_ns - s_prev.ts_ns) / (dt_seg * 1e9);
                    }
                    alpha = std::max(0.0, std::min(1.0, alpha));

                    // Fast LERP for quaternions
                    Eigen::Quaterniond q_pt;
                    q_pt.coeffs() = (1.0 - alpha) * s_prev.q.coeffs() + alpha * s_next.q.coeffs();
                    q_pt.normalize();

                    Eigen::Vector3d x_pt = (1.0 - alpha) * s_prev.p + alpha * s_next.p;

                    Eigen::Vector3d p_lever = p_l - L_l;
                    p_deskewed_l = q_pt * p_lever + L_l + x_pt;
                }
            } else {
                // Fallback to simple constant velocity model if trajectory is not available
                Eigen::Vector3d p_lever = p_l - L_l;
                Eigen::Vector3d rot_vec = dt * (R_l_c * imu_interpolator_->getInterpolatedAngularVel(pt_ts_ns));
                double theta = rot_vec.norm();
                Eigen::Vector3d p_lever_rotated = p_lever;
                if (theta > 1e-6) {
                    Eigen::Vector3d axis = rot_vec.normalized();
                    p_lever_rotated = p_lever * std::cos(theta) + axis.cross(p_lever) * std::sin(theta) +
                                     axis * axis.dot(p_lever) * (1.0 - std::cos(theta));
                }
                Eigen::Vector3d p_rotated = p_lever_rotated + L_l;
                p_deskewed_l = p_rotated + dt * v_l;
            }

            double disp = (p_deskewed_l - p_l).norm();
            total_displacement += disp;
            valid_points_count++;
            if (disp > max_displacement) {
                max_displacement = disp;
            }

            p.x = p_deskewed_l.x();
            p.y = p_deskewed_l.y();
            p.z = p_deskewed_l.z();
        }

        if (profiler) {
            double avg_displacement = (valid_points_count > 0) ? (total_displacement / valid_points_count) : 0.0;
            profiler->setDeskewMetrics(avg_displacement, max_displacement);
        }

        if (profiler) profiler->stopTimer("deskewing");
    }
}

void PerceptionPipeline::performClusteringAndMerging(
    const PointCloudPtr& obstacles_cloud, 
    std::vector<PointCloudPtr>& merged_clusters, 
    const PipelineConfig& config, 
    PerformanceProfiler* profiler) {

    if (!clusterer_) return;

    if (profiler) profiler->startTimer("clustering");
    std::vector<PointCloudPtr> clusters;
    clusterer_->cluster(obstacles_cloud, clusters);
    if (profiler) profiler->stopTimer("clustering");

    if (profiler) profiler->startTimer("merging");
    std::vector<PointCloudPtr> valid_candidates;
    valid_candidates.reserve(clusters.size());
    int min_cluster = config.min_cluster_size;
    for (auto& c : clusters) {
        if (c->size() >= static_cast<size_t>(min_cluster)) valid_candidates.push_back(c);
    }

    std::vector<bool> merged_flag(valid_candidates.size(), false);
    std::vector<Eigen::Vector3f> centroids(valid_candidates.size());
    
    for(size_t i = 0; i < valid_candidates.size(); ++i) {
        Eigen::Vector4f c4;
        pcl::compute3DCentroid(*valid_candidates[i], c4);
        centroids[i] = c4.head<3>();
    }

    const float merge_dist = static_cast<float>(config.merge_dist);
    const float MERGE_DIST_SQ = merge_dist * merge_dist; 
    for (size_t i = 0; i < valid_candidates.size(); ++i) {
        if (merged_flag[i]) continue;
        
        PointCloudPtr current_cloud = valid_candidates[i];
        bool has_merged = false;

        for (size_t j = i + 1; j < valid_candidates.size(); ++j) {
            if (merged_flag[j]) continue;
            float dx = centroids[i][0] - centroids[j][0];
            float dy = centroids[i][1] - centroids[j][1];

            if (dx*dx + dy*dy < MERGE_DIST_SQ) {
                if (!has_merged) {
                    current_cloud.reset(new PointCloud(*valid_candidates[i]));
                    has_merged = true;
                }
                *current_cloud += *valid_candidates[j];
                merged_flag[j] = true; 
            }
        }
        merged_clusters.push_back(current_cloud);
    }
    if (profiler) profiler->stopTimer("merging");
}

void PerceptionPipeline::aggregateDuplicates(
    const std::vector<Cone>& candidate_cones, 
    std::vector<Cone>& final_cones, 
    const PipelineConfig& config, 
    PerformanceProfiler* profiler) {

    if (profiler) profiler->startTimer("duplicate");
    const float tracking_match_dist = static_cast<float>(config.tracking_match_dist);
    const float MIN_DIST_SQ = tracking_match_dist * tracking_match_dist; 
    std::vector<bool> candidate_merged(candidate_cones.size(), false);

    for (size_t i = 0; i < candidate_cones.size(); ++i) {
        if (candidate_merged[i]) continue;
        
        Cone aggregated = candidate_cones[i];
        float sum_x = aggregated.x;
        float sum_y = aggregated.y;
        float sum_z = aggregated.z;
        int count = 1;

        for (size_t j = i + 1; j < candidate_cones.size(); ++j) {
            if (candidate_merged[j]) continue;
            
            float dist_sq = (candidate_cones[i].x - candidate_cones[j].x)*(candidate_cones[i].x - candidate_cones[j].x) +
                            (candidate_cones[i].y - candidate_cones[j].y)*(candidate_cones[i].y - candidate_cones[j].y);
            
            if (dist_sq < MIN_DIST_SQ) {
                sum_x += candidate_cones[j].x;
                sum_y += candidate_cones[j].y;
                sum_z += candidate_cones[j].z;
                count++;
                candidate_merged[j] = true;
            }
        }
        
        if (count > 1) {
            aggregated.x = sum_x / count;
            aggregated.y = sum_y / count;
            aggregated.z = sum_z / count;
            aggregated.range = std::sqrt(aggregated.x*aggregated.x + aggregated.y*aggregated.y);
            aggregated.bearing = std::atan2(aggregated.y, aggregated.x);
        }
        
        final_cones.push_back(aggregated);
    }
    if (profiler) profiler->stopTimer("duplicate");
}

} // namespace lidar_perception
