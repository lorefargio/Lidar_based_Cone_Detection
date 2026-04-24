#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_perception {

/**
 * @struct PointXYZIRT
 * @brief Customized LiDAR primitive with integrated temporal and structural metadata.
 * 
 * Extends the standard PCL PointXYZI with high-precision timestamps and laser ring IDs.
 * Optimized for SSE/AVX alignment to facilitate high-speed SIMD transformations.
 */
struct PointXYZIRT {
    PCL_ADD_POINT4D;              ///< Cartesian coordinates (padded for alignment).
    float intensity;              ///< Normalized return signal strength.
    double timestamp;             ///< Absolute Unix epoch timestamp (seconds).
    uint16_t ring;                ///< Index of the laser channel (bottom-to-top).
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW ///< SSE alignment mandate for Eigen operations.
} EIGEN_ALIGN16;

} // namespace lidar_perception

/**
 * @brief Registration macro for PCL integration.
 */
POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_perception::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)

namespace lidar_perception {
    using PointT = PointXYZIRT;
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = PointCloud::Ptr;
    using PointCloudConstPtr = PointCloud::ConstPtr;

    /**
     * @enum ConeColor
     * @brief Categorical labels for object classification.
     */
    enum class ConeColor { 
        UNKNOWN,      
        BLUE,         
        YELLOW,       
        ORANGE_BIG,   
        ORANGE_SMALL  
    };

    /**
     * @struct ClusterFeatures
     * @brief High-dimensional descriptor for volumetric candidate objects.
     * 
     * Encapsulates Cartesian position, polar orientation, bounding box dimensions, 
     * and PCA-derived shape metrics used for rigorous classification.
     */
    struct ClusterFeatures {
        int frame_id = 0;
        int cluster_id = 0;
        int point_count = 0;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float range = 0.0f;
        float bearing = 0.0f;
        float height = 0.0f;
        float width_max = 0.0f;
        float width_min = 0.0f;
        float aspect_ratio = 0.0f;
        float avg_intensity = 0.0f;
        float linearity = 0.0f;
        float planarity = 0.0f;
        float scattering = 0.0f;
        float verticality = 0.0f;
        float symmetry = 0.0f; 
    };

    /**
     * @struct Cone
     * @brief Perceptual primitive representing a validated object.
     */
    struct Cone {
        float x, y, z;        
        float range;          
        float bearing;        
        ConeColor color;      
        float confidence;     
        float height;         
        PointCloudPtr cloud;  
        ClusterFeatures features; 
    };
    } // namespace lidar_perception