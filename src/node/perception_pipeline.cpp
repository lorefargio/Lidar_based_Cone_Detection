#include "node/perception_pipeline.hpp"
#include "filtering/ground_remover_factory.hpp"
#include "clustering/clusterer_factory.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>

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
        imu_interpolator_->setLowPassCutoff(config.imu_lowpass_cutoff);
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

        // Rotate IMU angular velocity
        Eigen::Matrix4d T_im_l = tf_manager_->lookupTransformSafe(config.imu_frame, frame_id, target_ts_ns);
        Eigen::Matrix3d R_im_l = T_im_l.block<3, 3>(0, 0);
        
        // Optimisation: Precompute static lever arm translation vector outside the parallel loop
        Eigen::Vector3d t_im_l = T_im_l.block<3, 1>(0, 3);

        // Precompute gravity vector in LiDAR local frame
        Eigen::Vector3d g_l = R_w_l.transpose() * g_world_;

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

            // Interpolate IMU angular velocity
            Eigen::Vector3d omega_cam = imu_interpolator_->getInterpolatedAngularVel(pt_ts_ns);

            // 1. Lever-Arm Compensation
            Eigen::Vector3d p_l(p.x, p.y, p.z);
            Eigen::Vector3d p_im = R_im_l * p_l + t_im_l;

            // 2. Rotate the point about center of rotation
            Eigen::Vector3d rot_vec = dt * omega_cam;
            double theta = rot_vec.norm();
            Eigen::Vector3d p_im_rotated = p_im;
            if (theta > 1e-6) {
                Eigen::Vector3d axis = rot_vec.normalized();
                p_im_rotated = p_im * std::cos(theta) + axis.cross(p_im) * std::sin(theta) +
                               axis * axis.dot(p_im) * (1.0 - std::cos(theta));
            }

            // 3. Transform back to native LiDAR frame
            Eigen::Vector3d p_rotated = R_im_l.transpose() * (p_im_rotated - t_im_l);

            // 4. Second-order translation correction
            Eigen::Vector3d accel_cam = imu_interpolator_->getInterpolatedLinearAccel(pt_ts_ns);
            
            // Centripetal acceleration
            Eigen::Vector3d centripetal_im = omega_cam.cross(omega_cam.cross(t_im_l));
            Eigen::Vector3d total_accel_im = accel_cam + centripetal_im;
            
            Eigen::Vector3d accel_l = R_im_l.transpose() * total_accel_im;
            accel_l -= g_l;

            Eigen::Vector3d p_deskewed_l = p_rotated + dt * v_l + 0.5 * dt * dt * accel_l;

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
