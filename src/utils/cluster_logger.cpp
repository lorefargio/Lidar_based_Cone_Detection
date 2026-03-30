#include "utils/cluster_logger.hpp"
#include <iostream>
#include <iomanip>

namespace fs_perception {

ClusterLogger::ClusterLogger(const std::string& algo_name) : algo_name_(algo_name) {
    // Pre-reserve to avoid frequent reallocations
    data_.reserve(10000);
}

ClusterLogger::~ClusterLogger() {
}

void ClusterLogger::addCluster(const ClusterFeatures& features) {
    data_.push_back(features);
}

void ClusterLogger::saveToCSV(const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "Error: Unable to open CSV file for saving cluster data: " << filepath << std::endl;
        return;
    }

    // CSV Header
    out << "frame_id,cluster_id,point_count,x,y,z,range,bearing,height,width_max,width_min,aspect_ratio,avg_intensity,linearity,planarity,scattering,verticality,symmetry\n";

    // Data rows
    for (const auto& f : data_) {
        out << f.frame_id << ","
            << f.cluster_id << ","
            << f.point_count << ","
            << std::fixed << std::setprecision(4)
            << f.x << ","
            << f.y << ","
            << f.z << ","
            << f.range << ","
            << f.bearing << ","
            << f.height << ","
            << f.width_max << ","
            << f.width_min << ","
            << f.aspect_ratio << ","
            << f.avg_intensity << ","
            << f.linearity << ","
            << f.planarity << ","
            << f.scattering << ","
            << f.verticality << ","
            << f.symmetry << "\n";
    }

    out.close();
}

} // namespace fs_perception
