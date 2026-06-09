#include "node/visualization_bridge.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace lidar_perception {

VisualizationBridge::VisualizationBridge(rclcpp::Node* node) {
    pub_markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/cones_vis", rclcpp::SensorDataQoS());
    pub_cones_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/cones", rclcpp::SensorDataQoS());
    pub_cone_points_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/cone_points", rclcpp::SensorDataQoS());
    pub_lidar_viz_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/lidar_viz", rclcpp::SensorDataQoS());
    pub_obstacles_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/obstacles", rclcpp::SensorDataQoS());

    worker_thread_ = std::thread(&VisualizationBridge::workerLoop, this);
}

VisualizationBridge::~VisualizationBridge() {
    stop();
}

void VisualizationBridge::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_thread_) return;
        stop_thread_ = true;
    }
    cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void VisualizationBridge::enqueue(
    const std_msgs::msg::Header& header,
    const PointCloudPtr& processing_cloud,
    const std::vector<PointCloudPtr>& merged_clusters,
    const std::vector<Cone>& final_cones,
    const PipelineConfig& config) {

    std::lock_guard<std::mutex> lock(mutex_);
    debug_mode_ = config.debug;
    
    auto viz_data = std::make_unique<VizData>();
    viz_data->header = header;
    
    if (debug_mode_ && processing_cloud) {
        viz_data->cloud_viz.reset(new PointCloud(*processing_cloud));
    }
    viz_data->cones_cloud.reset(new PointCloud);
    viz_data->cones_cloud->reserve(final_cones.size());
    viz_data->cone_points.reset(new PointCloud);
    viz_data->cone_points->reserve(final_cones.size() * 30);
    
    viz_data->obstacles_cloud.reset(new PointCloud);
    size_t total_points = 0;
    for (const auto& cluster : merged_clusters) {
        total_points += cluster->size();
    }
    viz_data->obstacles_cloud->reserve(total_points);
    for (const auto& cluster : merged_clusters) {
        *(viz_data->obstacles_cloud) += *cluster;
    }
    
    if (debug_mode_) {
        auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
        markers->markers.reserve(final_cones.size() * 2 + 2);

        visualization_msgs::msg::Marker delete_template;
        delete_template.header = header;
        delete_template.action = visualization_msgs::msg::Marker::DELETEALL;
        delete_template.id = 0;
        delete_template.ns = "cones";
        markers->markers.push_back(delete_template);
        delete_template.ns = "distance_labels";
        markers->markers.push_back(delete_template);

        visualization_msgs::msg::Marker cone_template;
        cone_template.header = header;
        cone_template.ns = "cones";
        cone_template.type = visualization_msgs::msg::Marker::CYLINDER;
        cone_template.action = visualization_msgs::msg::Marker::ADD;
        cone_template.lifetime = rclcpp::Duration::from_seconds(0.2);
        cone_template.scale.x = 0.22; cone_template.scale.y = 0.22;
        cone_template.color.a = 1.0; 

        visualization_msgs::msg::Marker text_template;
        text_template.header = header;
        text_template.ns = "distance_labels";
        text_template.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_template.action = visualization_msgs::msg::Marker::ADD;
        text_template.lifetime = rclcpp::Duration::from_seconds(0.2);
        text_template.scale.z = 0.15;
        text_template.color.a = 1.0; text_template.color.r = 1.0; text_template.color.g = 1.0; text_template.color.b = 1.0;

        int id_counter = 1;
        for (const auto& cone : final_cones) {
            cone_template.id = id_counter;
            cone_template.pose.position.x = cone.x;
            cone_template.pose.position.y = cone.y;
            cone_template.pose.position.z = cone.z + (cone.height / 2.0); 
            cone_template.scale.z = std::max(0.1f, cone.height);
            
            if (cone.color == ConeColor::YELLOW) {
                cone_template.color.r = 1.0f; cone_template.color.g = 1.0f; cone_template.color.b = 0.0f;
            } else if (cone.color == ConeColor::BLUE) {
                cone_template.color.r = 0.0f; cone_template.color.g = 0.0f; cone_template.color.b = 1.0f;
            } else {
                cone_template.color.r = 0.5f; cone_template.color.g = 0.5f; cone_template.color.b = 0.5f;
            }
            markers->markers.push_back(cone_template);

            text_template.id = id_counter++;
            text_template.pose.position.x = cone.x;
            text_template.pose.position.y = cone.y;
            text_template.pose.position.z = cone.z + cone.height + 0.20;
            char dist_str[16];
            snprintf(dist_str, sizeof(dist_str), "%.1fm", cone.range);
            text_template.text = dist_str;
            markers->markers.push_back(text_template);
        }
        viz_data->markers = std::move(markers);
    }

    int cone_id = 1;
    for (const auto& cone : final_cones) {
        PointT p_out;
        p_out.x = cone.x; p_out.y = cone.y; p_out.z = cone.z;
        p_out.intensity = cone.range;
        viz_data->cones_cloud->push_back(p_out);
        
        if (cone.cloud) {
            for (auto& pt : cone.cloud->points) {
                pt.intensity = static_cast<float>(cone_id);
            }
            *(viz_data->cone_points) += *cone.cloud;
        }
        cone_id++;
    }
    
    queue_.push_back(std::move(viz_data));
    
    if (queue_.size() > 100) queue_.pop_front();
    cv_.notify_one();
}

void VisualizationBridge::workerLoop() {
    while (true) {
        std::unique_ptr<VizData> data;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !queue_.empty() || stop_thread_; });
            
            if (stop_thread_ && queue_.empty()) break;
            
            if (!queue_.empty()) {
                data = std::move(queue_.front());
                queue_.pop_front();
            }
        }

        if (!data) continue;

        if (data->markers && debug_mode_) {
            pub_markers_->publish(std::move(data->markers));
        }

        if (data->cloud_viz && debug_mode_) {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*(data->cloud_viz), *msg);
            msg->header = data->header;
            pub_lidar_viz_->publish(std::move(msg));
        }

        if (data->cones_cloud) {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*(data->cones_cloud), *msg);
            msg->header = data->header;
            pub_cones_->publish(std::move(msg));
        }

        if (data->cone_points) {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*(data->cone_points), *msg);
            msg->header = data->header;
            pub_cone_points_->publish(std::move(msg));
        }

        if (data->obstacles_cloud) {
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*(data->obstacles_cloud), *msg);
            msg->header = data->header;
            pub_obstacles_->publish(std::move(msg));
        }
    }
}

} // namespace lidar_perception
