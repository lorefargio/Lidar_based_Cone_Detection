#pragma once

#include "clustering/clusterer_interface.hpp"

namespace fs_perception {

class EuclideanClusterer : public ClustererInterface {
public:
    EuclideanClusterer(float cluster_tolerance = 0.35f, int min_cluster_size = 3, int max_cluster_size = 300);

    void cluster(const PointCloudPtr& cloud, 
                 std::vector<PointCloudPtr>& clusters) override;

private:
    float cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
};

} // namespace fs_perception