#pragma once

#include <string>
#include <chrono>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>

namespace fs_perception {

/**
 * @struct FrameData
 * @brief Stores latency and detection metrics for a single point cloud frame.
 */
struct FrameData {
    int frame_id;                 ///< Unique identifier for the processed frame.
    double conversion_ms = 0.0;     ///< Time taken for ROS to PCL conversion (ms).
    double deskewing_ms = 0.0;      ///< Time taken for the deskewing stage (ms).
    double ground_removal_ms = 0.0; ///< Time taken for the ground removal stage (ms).
    double clustering_ms = 0.0;     ///< Time taken for the clustering stage (ms).
    double merging_ms = 0.0;        ///< Time taken for the merging stage (ms).
    double estimation_ms = 0.0;     ///< Time taken for the estimation stage (ms).
    double duplicate_ms = 0.0;      ///< Time taken for the duplicate suppression stage (ms).
    double total_ms = 0.0;          ///< Total end-to-end processing time (ms).
    int cones_detected = 0;         ///< Number of cones successfully detected in the frame.
};

/**
 * @class PerformanceProfiler
 * @brief High-resolution timer and data collector for benchmarking the perception pipeline.
 * 
 * This class tracks the execution time of each stage of the pipeline and 
 * saves the collected data to a JSON file for offline analysis.
 */
class PerformanceProfiler {
public:
    /**
     * @brief Constructor for PerformanceProfiler.
     * @param algorithm_name Name of the currently used algorithm (for logging purposes).
     */
    PerformanceProfiler(const std::string& algorithm_name);

    /**
     * @brief Finalizes and closes the profiler, usually called when the node shuts down.
     */
    ~PerformanceProfiler();

    /**
     * @brief Prepares the data structure for a new incoming frame and starts the total frame timer.
     */
    void startFrame();

    /**
     * @brief Finalizes the current frame's timing and stores the detection results.
     * @param cones Number of cones detected in this frame.
     */
    void endFrame(int cones);

    /**
     * @brief Starts a high-resolution timer for a specific pipeline phase.
     * @param phase Name of the phase to profile (e.g., "ground_removal").
     */
    void startTimer(const std::string& phase);

    /**
     * @brief Stops a high-resolution timer for a specific pipeline phase.
     * @param phase Name of the phase to stop profiling.
     */
    void stopTimer(const std::string& phase);

    /**
     * @brief Gets the total latency of the last processed frame (ms).
     */
    double getLastFrameTotalMs() const {
        return frames_.empty() ? 0.0 : frames_.back().total_ms;
    }

    /**
     * @brief Serializes all collected frame data to a JSON file.
     * @param filepath Full system path where the JSON file will be saved.
     */
    void saveToJSON(const std::string& filepath);

private:
    std::string algorithm_name_;    ///< Algorithm descriptor for the session.
    int current_frame_id_;          ///< Sequential ID of the current frame being processed.
    FrameData current_frame_data_;  ///< Temporary storage for the current frame metrics.
    std::vector<FrameData> frames_; ///< List of data collected for every frame processed.
    
    /**
     * @brief Internal storage for various starting timestamps during profiling.
     */
    std::map<std::string, std::chrono::high_resolution_clock::time_point> timers_;
};

} // namespace fs_perception