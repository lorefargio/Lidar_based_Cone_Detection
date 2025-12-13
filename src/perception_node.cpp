#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <chrono>

#include "types.hpp"
#include "ground_remover.hpp"
#include "string_clusterer.hpp"
#include "cone_estimator.hpp"

using namespace std::chrono_literals;

class LidarPerceptionNode : public rclcpp::Node {
public:
    LidarPerceptionNode() : Node("lidar_perception_node") {
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

        RCLCPP_INFO(this->get_logger(), "Nodo Perception Avviato con Filtro Distanza (NMS)");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto start_total = std::chrono::high_resolution_clock::now();

        // 1. Conversione
        fs_perception::PointCloudPtr cloud(new fs_perception::PointCloud);
        pcl::fromROSMsg(*msg, *cloud);

        // 2. Ground Removal
        fs_perception::PointCloudPtr obstacles(new fs_perception::PointCloud);
        fs_perception::PointCloudPtr ground(new fs_perception::PointCloud);
        fs_perception::GroundRemover remover;
        remover.removeGround(cloud, obstacles, ground);

        // Debug Output
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground, ground_msg);
        ground_msg.header = msg->header;
        pub_ground_->publish(ground_msg);

        sensor_msgs::msg::PointCloud2 no_ground_msg;
        pcl::toROSMsg(*obstacles, no_ground_msg);
        no_ground_msg.header = msg->header;
        pub_no_ground_->publish(no_ground_msg);

        // 3. Clustering
        std::vector<fs_perception::PointCloudPtr> clusters;
        fs_perception::StringClusterer clusterer;
        clusterer.cluster(obstacles, clusters);

        std::vector<fs_perception::PointCloudPtr> merged_clusters;
        std::vector<bool> merged_flag(clusters.size(), false);

        for (size_t i = 0; i < clusters.size(); ++i) {
            if (merged_flag[i]) continue;

            // Calcola centroide A
            Eigen::Vector4f cA;
            pcl::compute3DCentroid(*clusters[i], cA);

            // Crea un nuovo super-cluster partendo da A
            fs_perception::PointCloudPtr super_cluster(new fs_perception::PointCloud(*clusters[i]));

            for (size_t j = i + 1; j < clusters.size(); ++j) {
                if (merged_flag[j]) continue;

                // Calcola centroide B
                Eigen::Vector4f cB;
                pcl::compute3DCentroid(*clusters[j], cB);

                // Distanza XY tra i centroidi (ignora Z per unire base e punta)
                float dist_xy = std::sqrt(std::pow(cA[0]-cB[0], 2) + std::pow(cA[1]-cB[1], 2));

                // Se i centri sono vicini (es. < 30cm), sono lo stesso cono spezzato
                if (dist_xy < 0.30f) {
                    *super_cluster += *clusters[j]; // Unisci le nuvole
                    merged_flag[j] = true; // Marca B come unito
                }
            }
            merged_clusters.push_back(super_cluster);
        }

        // 4. Stima Candidati
        fs_perception::ConeEstimator estimator;
        std::vector<fs_perception::Cone> candidate_cones;

        for (const auto& c : clusters) {
            auto cone = estimator.estimate(c);
            // Filtro confidence base
            if (cone.confidence > 0.5f) {
                candidate_cones.push_back(cone);
            }
        }

        // --- 5. FILTRO DISTANZA MINIMA (NMS Spaziale) ---
        // Se due coni sono più vicini di L metri, ne teniamo solo uno.
        // Questo evita "ghosting" o doppi rilevamenti dello stesso oggetto.
        
        const float MIN_CONE_DISTANCE = 0.8f; // 80cm: I coni FS non sono mai così vicini
        const float MIN_DIST_SQ = MIN_CONE_DISTANCE * MIN_CONE_DISTANCE;

        std::vector<fs_perception::Cone> final_cones;

        for (const auto& candidate : candidate_cones) {
            bool is_duplicate = false;
            
            // Controlla contro tutti quelli già accettati
            for (const auto& accepted : final_cones) {
                float dist_sq = (candidate.x - accepted.x)*(candidate.x - accepted.x) +
                                (candidate.y - accepted.y)*(candidate.y - accepted.y);
                
                if (dist_sq < MIN_DIST_SQ) {
                    is_duplicate = true;
                    // Qui potremmo fare logica avanzata (es. tenere quello con più confidence),
                    // ma per ora "chi prima arriva meglio alloggia" funziona bene col LiDAR.
                    break;
                }
            }

            if (!is_duplicate) {
                final_cones.push_back(candidate);
            }
        }

        // --- 6. VISUALIZZAZIONE & OUTPUT ---
        visualization_msgs::msg::MarkerArray markers;
        fs_perception::PointCloud cones_cloud_out; 

        // Fix Ghosting RViz
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_marker.header = msg->header;
        delete_marker.ns = "cones";
        delete_marker.id = 0;
        markers.markers.push_back(delete_marker);

        int id_counter = 1;

        for (const auto& cone : final_cones) {
            // Marker
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

            // Colore (Logica Semplificata: Blu=Geometrico)
            if (cone.color == fs_perception::ConeColor::BLUE) {
                m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f;
            } else if (cone.color == fs_perception::ConeColor::YELLOW) {
                m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 0.0f;
            } else {
                // Default BLU per geometria pura
                m.color.b = 1.0f; m.color.r = 0.0f; m.color.g = 0.0f; 
            }

            markers.markers.push_back(m);

            // PointCloud
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

        // Profiler
        auto end_total = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_total - start_total).count();
        
        static int frame = 0;
        if (++frame % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "Time: %ld ms | Candidates: %ld | Filtered: %ld", 
                dur, candidate_cones.size(), final_cones.size());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}