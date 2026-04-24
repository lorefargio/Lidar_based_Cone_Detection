#pragma once

#include <string>
#include <chrono>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>

namespace lidar_perception {

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
 * @brief High-resolution instrumentation utility for pipeline latency analysis.
 * 
 * This class provides high-precision timing for sequential pipeline stages, enabling
 * deterministic analysis of P99 latency and probability density functions (PDF).
 * Data is accumulated in-memory to prevent filesystem I/O interference during critical paths.
 */
class PerformanceProfiler {
public:
    /**
     * @brief Construct a new Performance Profiler object.
     * @param algorithm_name Descriptor for the current configuration (used in JSON metadata).
     */
    PerformanceProfiler(const std::string& algorithm_name);

    /**
     * @brief Finalizes data collection and ensures memory safety on shutdown.
     */
    ~PerformanceProfiler();

    /**
     * @brief Resets frame-specific metrics and initializes the global frame timer.
     */
    void startFrame();

    /**
     * @brief Records the final latency for the current frame and increments detection metrics.
     * @param cones Number of validated cones in the current frame.
     */
    void endFrame(int cones);

    /**
     * @brief Starts a high-resolution timer for a specific pipeline phase.
     * @param phase Identifier for the stage (e.g., "clustering", "ground_removal").
     */
    void startTimer(const std::string& phase);

    /**
     * @brief Stops the timer for a specific phase and calculates the elapsed duration in milliseconds.
     * @param phase Identifier for the stage to stop profiling.
     */
    void stopTimer(const std::string& phase);

    /**
     * @brief Retrieves the end-to-end latency of the most recently processed frame.
     * @return double Elapsed time in milliseconds.
     */
    double getLastFrameTotalMs() const {
        return frames_.empty() ? 0.0 : frames_.back().total_ms;
    }

    /**
     * @brief Serializes the accumulated frame data to a structured JSON file for offline PDF/P99 analysis.
     * @param filepath Target system path for the output file.
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

} // namespace lidar_perception