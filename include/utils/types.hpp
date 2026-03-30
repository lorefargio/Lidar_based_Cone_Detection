#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs_perception {

/**
 * @struct PointXYZIRT
 * @brief Custom point type representing 3D coordinates, Intensity, Timestamp, and Ring.
 * 
 * Specifically designed for LiDAR sensors (e.g., Hesai Pandar) that provide 
 * additional per-point metadata for advanced processing.
 */
struct PointXYZIRT {
    PCL_ADD_POINT4D;              ///< XYZ coordinates + padding for SSE alignment.
    float intensity;              ///< Intensity value of the point return.
    double timestamp;             ///< Precise acquisition timestamp of the point.
    uint16_t ring;                ///< Laser ring ID from the sensor scan.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW ///< Ensures correct memory alignment for Eigen.
} EIGEN_ALIGN16;

} // namespace fs_perception

/**
 * @brief Registration of the PointXYZIRT struct into the PCL library.
 */
POINT_CLOUD_REGISTER_POINT_STRUCT(fs_perception::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)

namespace fs_perception {
    using PointT = PointXYZIRT;                           ///< Standard point type for the pipeline.
    using PointCloud = pcl::PointCloud<PointT>;            ///< Standard point cloud type.
    using PointCloudPtr = PointCloud::Ptr;                ///< Pointer to a point cloud.
    using PointCloudConstPtr = PointCloud::ConstPtr;      ///< Constant pointer to a point cloud.

    /**
     * @enum ConeColor
     * @brief Supported cone colors for classification.
     */
    enum class ConeColor { 
        UNKNOWN,      ///< Unclassified color.
        BLUE,         ///< Blue cone (e.g., left track boundary).
        YELLOW,       ///< Yellow cone (e.g., right track boundary).
        ORANGE_BIG,   ///< Big orange cone (e.g., start/finish line).
        ORANGE_SMALL  ///< Small orange cone (e.g., obstacles or zone markers).
    };

    /**
     * @struct ClusterFeatures
     * @brief Detailed geometric and statistical features of a 3D cluster.
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
        float symmetry = 0.0f; // max_width / min_width
    };

    /**
     * @struct Cone
     * @brief Representation of a detected cone with its estimated properties.
     */
    struct Cone {
        float x, y, z;        ///< 3D position of the cone in the sensor frame (meters).
        float range;          ///< Radial distance from the sensor (meters).
        float bearing;        ///< Angular position in the XY plane (radians).
        ConeColor color;      ///< Estimated color category of the cone.
        float confidence;     ///< Confidence score of the detection (0.0 to 1.0).
        float height;         ///< Estimated height of the cone (meters), used for debug.
        PointCloudPtr cloud;  ///< Original points belonging to this cone cluster.
        ClusterFeatures features; ///< Extracted characteristics of the cluster.
    };
}