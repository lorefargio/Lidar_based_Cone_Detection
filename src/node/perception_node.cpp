#include "utils/types.hpp"

#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>      // <--- IMPORTANTE: include l'implementazione della base
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/centroid.hpp>
#include <chrono>
#include <memory>
#include <fstream>
#include <filesystem>


#include "filtering/ground_remover_interface.hpp"
#include "filtering/bin_based_ground_remover.hpp"
#include "filtering/slope_based_ground_remover.hpp"
#include "filtering/patchwork_pp_ground_remover.hpp"
#include "filtering/official_patchwork_pp_ground_remover.hpp"
#include "clustering/clusterer_interface.hpp"
#include "clustering/string_clusterer.hpp"
#include "clustering/euclidean_clusterer.hpp"
#include "clustering/grid_clusterer.hpp"
#include "clustering/dbscan_clusterer.hpp"
#include "clustering/adaptive_dbscan_clusterer.hpp"
#include "clustering/voxel_connected_components.hpp"
#include "estimation/estimator_interface.hpp"
#include "estimation/rule_based_estimator.hpp"
#include "estimation/model_fitting_estimator.hpp"
#include "utils/performance_profiler.hpp"

using namespace std::chrono_literals;

namespace pcl {
  template class PCLBase<fs_perception::PointT>;
  template class VoxelGrid<fs_perception::PointT>;
}
class LidarPerceptionNode : public rclcpp::Node {
public:
    LidarPerceptionNode() : Node("lidar_perception_node") {
        // --- Parametri ---
        this->declare_parameter<std::string>("clustering_algorithm", "grid");
        std::string algo = this->get_parameter("clustering_algorithm").as_string();

        if (algo == "euclidean") {
            clusterer_ = std::make_unique<fs_perception::EuclideanClusterer>(0.35f, 3, 300);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: EUCLIDEAN (KD-Tree)");
        } else if (algo == "string") {
            clusterer_ = std::make_unique<fs_perception::StringClusterer>();
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: STRING (Linear)");
        } else if (algo == "dbscan") {
            clusterer_ = std::make_unique<fs_perception::DBSCANClusterer>(0.30f, 3, 3, 500);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: DBSCAN (KD-Tree)");
        } else if (algo == "hdbscan") {
            clusterer_ = std::make_unique<fs_perception::AdaptiveDBSCANClusterer>(0.15f, 0.015f, 3, 3, 500);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: ADAPTIVE-DBSCAN (HDBSCAN Variant)");
        } else if (algo == "voxel") {
            clusterer_ = std::make_unique<fs_perception::VoxelConnectedComponents>(0.15f, 3, 500);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: VOXEL CONNECTED COMPONENTS (3D Grid)");
        } else {
            // Default: Grid (Fastest & robust for 20Hz)
            clusterer_ = std::make_unique<fs_perception::GridClusterer>(0.20f, 25.0f, 3, 300);
            RCLCPP_INFO(this->get_logger(), "Clustering Algorithm: GRID (2D Connected Components)");
        }

        // --- Ground Removal Parametrizzazione ---
        this->declare_parameter<std::string>("ground_remover_type", "slope_based");
        std::string gr_type = this->get_parameter("ground_remover_type").as_string();

        if (gr_type == "bin_based") {
            ground_remover_ = std::make_unique<fs_perception::BinBasedGroundRemover>();
            RCLCPP_INFO(this->get_logger(), "Ground Removal: BIN-BASED (Fast)");
        } else if (gr_type == "slope_based") {
            ground_remover_ = std::make_unique<fs_perception::SlopeBasedGroundRemover>();
            RCLCPP_INFO(this->get_logger(), "Ground Removal: SLOPE-BASED (Precise)");
        } else {
            ground_remover_ = std::make_unique<fs_perception::SlopeBasedGroundRemover>();
            RCLCPP_INFO(this->get_logger(), "Ground Removal: Defaulting to SLOPE-BASED");
        }

        this->declare_parameter<bool>("use_voxel_filter", false);
        this->declare_parameter<double>("voxel_size", 0.05);

        this->declare_parameter<std::string>("estimator_type", "rule_based");
        std::string est_type = this->get_parameter("estimator_type").as_string();

        this->declare_parameter<double>("dynamic_width_decay", 0.005);
        this->declare_parameter<int>("min_points_at_10m", 10);
        this->declare_parameter<double>("pca_max_linearity", 0.8);
        this->declare_parameter<double>("pca_max_planarity", 0.8);
        this->declare_parameter<double>("pca_min_scatter", 0.05);

        if (est_type == "rule_based") {
            fs_perception::RuleBasedEstimator::Config config;
            config.dynamic_width_decay = this->get_parameter("dynamic_width_decay").as_double();
            config.min_points_at_10m = this->get_parameter("min_points_at_10m").as_int();
            config.max_linearity = this->get_parameter("pca_max_linearity").as_double();
            config.max_planarity = this->get_parameter("pca_max_planarity").as_double();
            config.min_scatter = this->get_parameter("pca_min_scatter").as_double();

            estimator_ = std::make_unique<fs_perception::RuleBasedEstimator>(config);
            RCLCPP_INFO(this->get_logger(), "Estimator Algorithm: RULE-BASED (Dynamic Thresholds)");
        } else if (est_type == "ransac") {
            estimator_ = std::make_unique<fs_perception::ModelFittingEstimator>();
            RCLCPP_INFO(this->get_logger(), "Estimator Algorithm: RANSAC MODEL FITTING (Cylinder)");
        } else {
            estimator_ = std::make_unique<fs_perception::RuleBasedEstimator>();
            RCLCPP_INFO(this->get_logger(), "Estimator Algorithm: Defaulting to RULE-BASED");
        }

        // --- Setup Profiler JSON ---
        std::string log_dir = "/home/lore/lidar_ws/log_profiler/";
        std::filesystem::create_directories(log_dir); // Ensure directory exists
        
        profiler_ = std::make_unique<fs_perception::PerformanceProfiler>(algo);
        json_file_path_ = log_dir + "profiler_" + algo + ".json";
        RCLCPP_INFO(this->get_logger(), "Profiler inizializzato. Salverà su: %s", json_file_path_.c_str());

        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", qos, 
            std::bind(&LidarPerceptionNode::callback, this, std::placeholders::_1));

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/cones_vis", 10);
        pub_cones_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/cones", 10);
        pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_debug", 10);
        pub_no_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/no_ground", 10);

        RCLCPP_INFO(this->get_logger(), "Nodo Perception Avviato.");
    }

    ~LidarPerceptionNode() {
        if (profiler_) {
            profiler_->saveToJSON(json_file_path_);
            RCLCPP_INFO(this->get_logger(), "Profilazione salvata su JSON in chiusura.");
        }
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        frame_counter_++;
        profiler_->startFrame();

        // 1. Conversione
        fs_perception::PointCloudPtr raw_cloud(new fs_perception::PointCloud);
        pcl::fromROSMsg(*msg, *raw_cloud);
        
        if (raw_cloud->empty()) return;

        // 2. Ground Removal (Su nuvola a piena risoluzione per massima precisione)
        profiler_->startTimer("ground_removal");
        obstacles_str_->clear();
        ground_str_->clear();

        ground_remover_->removeGround(raw_cloud, obstacles_str_, ground_str_);
        profiler_->stopTimer("ground_removal");

        // 2.1 Voxel Grid Downsampling (Solo sugli ostacoli per velocizzare Clustering ed Estimation)
        if (this->get_parameter("use_voxel_filter").as_bool()) {
            double leaf_size = this->get_parameter("voxel_size").as_double();
            pcl::VoxelGrid<fs_perception::PointT> voxel_grid;
            voxel_grid.setInputCloud(obstacles_str_);
            voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
            
            fs_perception::PointCloudPtr filtered_obstacles(new fs_perception::PointCloud);
            voxel_grid.filter(*filtered_obstacles);
            obstacles_str_ = filtered_obstacles;
        }

        // Debug Output
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_str_, ground_msg);
        ground_msg.header = msg->header;
        pub_ground_->publish(ground_msg);

        sensor_msgs::msg::PointCloud2 no_ground_msg;
        pcl::toROSMsg(*obstacles_str_, no_ground_msg);
        no_ground_msg.header = msg->header;
        pub_no_ground_->publish(no_ground_msg);

        // 3. Clustering
        profiler_->startTimer("clustering");
        std::vector<fs_perception::PointCloudPtr> clusters;
        clusterer_->cluster(obstacles_str_, clusters);

        std::vector<fs_perception::PointCloudPtr> merged_clusters;
        std::vector<bool> merged_flag(clusters.size(), false);
        
        // Centroidi con aligned_allocator per evitare Segfault (Eigen alignment)
        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids(clusters.size());
        
        for(size_t i=0; i<clusters.size(); ++i) {
            pcl::compute3DCentroid(*clusters[i], centroids[i]);
        }

        const float MERGE_DIST_SQ = 0.25f * 0.25f; // 25cm (era 30cm)
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (merged_flag[i]) continue;
            fs_perception::PointCloudPtr super_cluster(new fs_perception::PointCloud(*clusters[i]));

            for (size_t j = i + 1; j < clusters.size(); ++j) {
                if (merged_flag[j]) continue;
                float dx = centroids[i][0] -centroids[j][0] ;
                float dy = centroids[i][1] -centroids[j][1] ;

                if (dx*dx +dy*dy < MERGE_DIST_SQ) {
                    *super_cluster += *clusters[j];
                    merged_flag[j] = true; 
                }
            }
            merged_clusters.push_back(super_cluster);
        }
        profiler_->stopTimer("clustering");

        // 4. Stima Candidati & Filtraggio
        profiler_->startTimer("estimation");
        std::vector<fs_perception::Cone> candidate_cones;

        for (const auto& c : merged_clusters) {
            auto cone = estimator_->estimate(c);
            if (cone.confidence > 0.5f) {
                candidate_cones.push_back(cone);
            }
        }
        
        // Ridotta soglia di duplicazione: in FS i coni possono essere vicini (es. 70-80cm)
        const float MIN_DIST_SQ = 0.4f * 0.4f; // 40cm (era 80cm)
        std::vector<fs_perception::Cone> final_cones;

        for (const auto& candidate : candidate_cones) {
            bool is_duplicate = false;
            for (const auto& accepted : final_cones) {
                float dist_sq = (candidate.x - accepted.x)*(candidate.x - accepted.x) +
                                (candidate.y - accepted.y)*(candidate.y - accepted.y);
                if (dist_sq < MIN_DIST_SQ) {
                    is_duplicate = true;
                    break;
                }
            }
            if (!is_duplicate) {
                final_cones.push_back(candidate);
            }
        }

        profiler_->stopTimer("estimation");

        // 5. VISUALIZZAZIONE & OUTPUT
        visualization_msgs::msg::MarkerArray markers;
        fs_perception::PointCloud cones_cloud_out; 

        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.header = msg->header;
        delete_marker.ns = "cones";
        delete_marker.id = 0;
        markers.markers.push_back(delete_marker);

        int id_counter = 1;
        for (const auto& cone : final_cones) {
            visualization_msgs::msg::Marker m;
            m.header = msg->header;
            m.ns = "cones";
            m.id = id_counter++;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.lifetime = rclcpp::Duration::from_seconds(0.2); 

            m.pose.position.x = cone.x;
            m.pose.position.y = cone.y;
            m.pose.position.z = cone.z + (cone.height/2.0); 
            m.scale.x = 0.2; m.scale.y = 0.2; m.scale.z = cone.height;
            m.color.a = 1.0;
            
            if (cone.color == fs_perception::ConeColor::BLUE) { m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f; }
            else if (cone.color == fs_perception::ConeColor::YELLOW) { m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f; }
            else { m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f; }

            markers.markers.push_back(m);

            fs_perception::PointT p_out;
            p_out.x = cone.x; p_out.y = cone.y; p_out.z = cone.z;
            p_out.intensity = 10.0f; 
            cones_cloud_out.push_back(p_out);
        }

        pub_markers_->publish(markers);

        sensor_msgs::msg::PointCloud2 cones_msg;
        pcl::toROSMsg(cones_cloud_out, cones_msg);
        cones_msg.header = msg->header;
        pub_cones_->publish(cones_msg);

        // 6. LOGGING PROFILER
        profiler_->endFrame(final_cones.size());

        if (frame_counter_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "Frame: %d | Frame processato (salvataggio su JSON in corso)", frame_counter_);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_;

    fs_perception::PointCloudPtr cloud_str_{new fs_perception::PointCloud};
    fs_perception::PointCloudPtr obstacles_str_{new fs_perception::PointCloud};
    fs_perception::PointCloudPtr ground_str_{new fs_perception::PointCloud};
    std::unique_ptr<fs_perception::GroundRemoverInterface> ground_remover_;
    std::unique_ptr<fs_perception::ClustererInterface> clusterer_;
    std::unique_ptr<fs_perception::EstimatorInterface> estimator_;
    
    std::unique_ptr<fs_perception::PerformanceProfiler> profiler_;
    std::string json_file_path_;
    int frame_counter_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}