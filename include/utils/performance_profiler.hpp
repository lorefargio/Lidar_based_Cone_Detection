#pragma once

#include <string>
#include <chrono>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>

namespace fs_perception {

struct FrameData {
    int frame_id;
    double ground_removal_ms = 0.0;
    double clustering_ms = 0.0;
    double estimation_ms = 0.0;
    double total_ms = 0.0;
    int cones_detected = 0;
};

class PerformanceProfiler {
public:
    PerformanceProfiler(const std::string& algorithm_name);
    ~PerformanceProfiler();

    void startFrame();
    void endFrame(int cones);

    void startTimer(const std::string& phase);
    void stopTimer(const std::string& phase);

    void saveToJSON(const std::string& filepath);

private:
    std::string algorithm_name_;
    int current_frame_id_;
    FrameData current_frame_data_;
    std::vector<FrameData> frames_;
    
    std::map<std::string, std::chrono::high_resolution_clock::time_point> timers_;
};

} // namespace fs_perception
