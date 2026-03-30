#pragma once

#include <vector>
#include <string>
#include <fstream>
#include "utils/types.hpp"

namespace fs_perception {

/**
 * @class ClusterLogger
 * @brief Records geometric and statistical features of detected cones to a CSV file.
 * 
 * This utility allows for offline analysis of cone characteristics to help
 * fine-tune clustering and classification parameters.
 */
class ClusterLogger {
public:
    /**
     * @brief Constructor for ClusterLogger.
     * @param algo_name Name of the clustering algorithm being used (for file naming).
     */
    ClusterLogger(const std::string& algo_name);

    /**
     * @brief Destructor. Ensures the file is flushed and closed.
     */
    ~ClusterLogger();

    /**
     * @brief Adds a cluster's features to the internal buffer.
     * @param features The extracted features of a detected cone.
     */
    void addCluster(const ClusterFeatures& features);

    /**
     * @brief Saves all collected cluster data to a CSV file.
     * @param filepath System path where the CSV will be saved.
     */
    void saveToCSV(const std::string& filepath);

private:
    std::string algo_name_;              ///< Current algorithm name.
    std::vector<ClusterFeatures> data_;  ///< Buffer for cluster features.
};

} // namespace fs_perception
