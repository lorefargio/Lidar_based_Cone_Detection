#pragma once

#include <vector>
#include <string>
#include <fstream>
#include "utils/types.hpp"

namespace fs_perception {

/**
 * @class ClusterLogger
 * @brief Diagnostic utility for recording high-dimensional cluster features.
 * 
 * Facilitates offline experimental analysis by exporting geometric and statistical 
 * descriptors of detected clusters to a structured CSV format.
 */
class ClusterLogger {
public:
    /**
     * @brief Construct a new Cluster Logger object.
     * @param algo_name Descriptor for the clustering algorithm under test.
     */
    ClusterLogger(const std::string& algo_name);

    /**
     * @brief Flushes pending data and ensures file integrity on shutdown.
     */
    ~ClusterLogger();

    /**
     * @brief Buffers a cluster's feature set for eventual serialization.
     * @param features Struct containing geometric, intensity, and PCA descriptors.
     */
    void addCluster(const ClusterFeatures& features);

    /**
     * @brief Serializes all buffered cluster features to a CSV file.
     * @param filepath Target system path for the output CSV.
     */
    void saveToCSV(const std::string& filepath);

private:
    std::string algo_name_;              ///< Current algorithm name.
    std::vector<ClusterFeatures> data_;  ///< Buffer for cluster features.
};

} // namespace fs_perception
