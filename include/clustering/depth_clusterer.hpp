#pragma once

#include "utils/types.hpp"
#include "clustering/clusterer_interface.hpp"
#include <vector>

namespace fs_perception {

/**
 * @class DepthClusterer
 * @brief State-of-the-Art 2D Range-Image-based Clustering (Depth-Clustering).
 * 
 * This algorithm evolves the 1D StringClusterer into a 2D topological grouping strategy.
 * It projects 3D points into a Range Image (rows = rings, cols = azimuthal sequence)
 * and performs a 4-connected BFS segmentation.
 * 
 * Connectivity is determined by the angle $\beta$ between two laser beams and the 
 * line connecting their endpoints, making the algorithm scale-invariant.
 * 
 * @cite Bogoslavskyi, I., & Stachniss, C. (2016). "Fast range image-based segmentation of sparse 3D Lidar scans."
 */
class DepthClusterer : public ClustererInterface {
public:
    /**
     * @struct Config
     * @brief Configuration for the Depth-Clustering logic.
     */
    struct Config {
        float theta_threshold_deg = 8.0f;  ///< Min angle beta (degrees) to consider two points connected.
        int num_rings = 32;               ///< Max expected rings (auto-detected if higher).
        int num_cols = 2048;              ///< Higher horizontal resolution to avoid collisions.
        int min_cluster_size = 3;         ///< Min points for a valid cone.
        int max_cluster_size = 300;       ///< Max points for a valid cone.
    };

    /**
     * @brief Default constructor.
     */
    DepthClusterer();

    /**
     * @brief Constructor with custom configuration.
     */
    explicit DepthClusterer(const Config& config);

    /**
     * @brief Performs 2D range-image-based segmentation.
     * @param cloud Input non-ground point cloud.
     * @param clusters Identified object clusters.
     */
    void cluster(const PointCloudPtr& cloud, std::vector<PointCloudPtr>& clusters) override;

private:
    Config config_;
    float theta_threshold_rad_;
    float hor_res_rad_; ///< Horizontal angular resolution.
};

} // namespace fs_perception
