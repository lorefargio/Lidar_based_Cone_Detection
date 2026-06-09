#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "utils/types.hpp"
#include "node/pipeline_config.hpp"
#include "filtering/ground_remover_interface.hpp"
#include "clustering/clusterer_interface.hpp"
#include "estimation/cone_estimator.hpp"
#include "utils/transform_manager.hpp"
#include "utils/imu_interpolator.hpp"
#include "utils/performance_profiler.hpp"
#include "utils/cluster_logger.hpp"

namespace lidar_perception {

/**
 * @class PerceptionPipeline
 * @brief Facade class encapsulating all point cloud algorithm stages, independent of the ROS 2 node.
 */
class PerceptionPipeline {
public:
    PerceptionPipeline();
    ~PerceptionPipeline();

    /**
     * @brief Initialize all pipeline stages using factories and parameters.
     */
    void initialize(const PipelineConfig& config, const rclcpp::Clock::SharedPtr& clock);

    /**
     * @brief Runs the complete perception pipeline sequentially.
     */
    std::vector<Cone> run(
        PointCloudPtr& input_cloud, 
        uint64_t target_ts_ns, 
        const std::string& frame_id, 
        int frame_counter, 
        const PipelineConfig& config, 
        ClusterLogger* cluster_logger, 
        PerformanceProfiler* profiler,
        PointCloudPtr& obstacles_out,
        PointCloudPtr& ground_out,
        std::vector<PointCloudPtr>& merged_clusters_out);

    void preProcess(const PointCloudPtr& cloud, const PipelineConfig& config);

    // Module accessors for IMU data injection
    fs_fusion::ImuInterpolator* getImuInterpolator() { return imu_interpolator_.get(); }
    fs_fusion::TransformManager* getTransformManager() { return tf_manager_.get(); }

private:
    
    void performDeskewing(
        uint64_t target_ts_ns, 
        const std::string& frame_id, 
        PointCloudPtr& processing_cloud, 
        const PipelineConfig& config, 
        PerformanceProfiler* profiler);
        
    void performClusteringAndMerging(
        const PointCloudPtr& obstacles_cloud, 
        std::vector<PointCloudPtr>& merged_clusters, 
        const PipelineConfig& config, 
        PerformanceProfiler* profiler);
        
    void estimateCones(
        const std::vector<PointCloudPtr>& merged_clusters, 
        std::vector<Cone>& candidate_cones, 
        int frame_counter, 
        ClusterLogger* cluster_logger, 
        PerformanceProfiler* profiler);
        
    void aggregateDuplicates(
        const std::vector<Cone>& candidate_cones, 
        std::vector<Cone>& final_cones, 
        const PipelineConfig& config, 
        PerformanceProfiler* profiler);

    std::unique_ptr<GroundRemoverInterface> ground_remover_;
    std::unique_ptr<ClustererInterface> clusterer_;
    std::unique_ptr<ConeEstimator> estimator_;
    std::unique_ptr<fs_fusion::TransformManager> tf_manager_;
    std::unique_ptr<fs_fusion::ImuInterpolator> imu_interpolator_;

    Eigen::Vector3d g_world_;
};

} // namespace lidar_perception
