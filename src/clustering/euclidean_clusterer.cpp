#include "clustering/euclidean_clusterer.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

namespace lidar_perception {

EuclideanClusterer::EuclideanClusterer(float cluster_tolerance, int min_cluster_size, int max_cluster_size)
    : cluster_tolerance_(cluster_tolerance), 
      min_cluster_size_(min_cluster_size), 
      max_cluster_size_(max_cluster_size) {}

void EuclideanClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    PointCloudPtr processed_cloud = cloud;

    // Optimization: Internal VoxelGrid downsampling if the cloud is too dense
    // This significantly speeds up KD-Tree construction and neighbor searching.
    if (cloud->size() > 5000) {
        PointCloudPtr filtered_cloud(new PointCloud);
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.04f, 0.04f, 0.04f); // 4cm voxel
        vg.filter(*filtered_cloud);
        processed_cloud = filtered_cloud;
    }

    if (processed_cloud->empty()) return;

    // Initialize the search tree for efficient neighbor lookup
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(processed_cloud);

    // Using the standard PCL Euclidean extraction method
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(processed_cloud);
    ec.extract(cluster_indices);

    // Convert indices back to point clouds for each cluster
    for (const auto& indices : cluster_indices) {
        PointCloudPtr cluster_cloud(new PointCloud);
        cluster_cloud->points.reserve(indices.indices.size());
        for (const auto& idx : indices.indices) {
            cluster_cloud->push_back(processed_cloud->points[idx]);
        }
        cluster_cloud->width = static_cast<uint32_t>(cluster_cloud->size());
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        clusters.push_back(cluster_cloud);
    }
}

} // namespace lidar_perception
