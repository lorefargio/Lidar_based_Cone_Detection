#pragma once

#include "clustering/clusterer_interface.hpp"
#include <pcl/search/kdtree.h>
#include <vector>

namespace fs_perception {

class DBSCANClusterer : public ClustererInterface {
public:
    DBSCANClusterer(float eps = 0.30f, int min_pts = 3, int min_cluster_size = 3, int max_cluster_size = 500);
    ~DBSCANClusterer() override = default;

    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    float eps_;
    int min_pts_;
    int min_cluster_size_;
    int max_cluster_size_;
};

} // namespace fs_perception
