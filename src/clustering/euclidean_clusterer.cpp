#include "clustering/euclidean_clusterer.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace fs_perception {

EuclideanClusterer::EuclideanClusterer(float cluster_tolerance, int min_cluster_size, int max_cluster_size)
    : cluster_tolerance_(cluster_tolerance), 
      min_cluster_size_(min_cluster_size), 
      max_cluster_size_(max_cluster_size) {}

void EuclideanClusterer::cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) {
    if (cloud->empty()) return;

    // KD-Tree per la ricerca veloce dei vicini
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_); // in metri
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Converte gli indici in PointCloud
    for (const auto& indices : cluster_indices) {
        PointCloudPtr cluster_cloud(new PointCloud);
        for (const auto& idx : indices.indices) {
            cluster_cloud->push_back(cloud->points[idx]);
        }
        cluster_cloud->width = cluster_cloud->size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        clusters.push_back(cluster_cloud);
    }
}

} // namespace fs_perception