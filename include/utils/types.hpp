#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs_perception {

// Definizione punto custom per Hesai Pandar (XYZ + Intensity + Timestamp + Ring)
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace fs_perception

// Registrazione PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(fs_perception::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)

namespace fs_perception {
    using PointT = PointXYZIRT;
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = PointCloud::Ptr;
    using PointCloudConstPtr = PointCloud::ConstPtr;

    enum class ConeColor { UNKNOWN, BLUE, YELLOW, ORANGE_BIG, ORANGE_SMALL };

    struct Cone {
        float x, y, z;
        ConeColor color;
        float confidence;
        float height; // Per debug
    };
}
