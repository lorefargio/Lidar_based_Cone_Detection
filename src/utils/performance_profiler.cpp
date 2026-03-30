#include "utils/performance_profiler.hpp"
#include <algorithm>
#include <numeric>
#include <iomanip>

namespace fs_perception {

PerformanceProfiler::PerformanceProfiler(const std::string& algorithm_name) 
    : algorithm_name_(algorithm_name), current_frame_id_(0) {
    // Pre-allocate memory to avoid overhead during critical path processing
    frames_.reserve(5000); 
}

PerformanceProfiler::~PerformanceProfiler() {
}

void PerformanceProfiler::startFrame() {
    current_frame_id_++;
    current_frame_data_ = FrameData();
    current_frame_data_.frame_id = current_frame_id_;
    startTimer("total");
}

void PerformanceProfiler::startTimer(const std::string& phase) {
    timers_[phase] = std::chrono::high_resolution_clock::now();
}

void PerformanceProfiler::stopTimer(const std::string& phase) {
    auto end_time = std::chrono::high_resolution_clock::now();
    if (timers_.find(phase) != timers_.end()) {
        double elapsed = std::chrono::duration<double, std::milli>(end_time - timers_[phase]).count();
        
        // Map the measured time to the corresponding frame data field
        if (phase == "conversion") {
            current_frame_data_.conversion_ms = elapsed;
        } else if (phase == "deskewing") {
            current_frame_data_.deskewing_ms = elapsed;
        } else if (phase == "ground_removal") {
            current_frame_data_.ground_removal_ms = elapsed;
        } else if (phase == "clustering") {
            current_frame_data_.clustering_ms = elapsed;
        } else if (phase == "merging") {
            current_frame_data_.merging_ms = elapsed;
        } else if (phase == "estimation") {
            current_frame_data_.estimation_ms = elapsed;
        } else if (phase == "duplicate") {
            current_frame_data_.duplicate_ms = elapsed;
        } else if (phase == "total") {
            current_frame_data_.total_ms = elapsed;
        }
    }
}

void PerformanceProfiler::endFrame(int cones) {
    stopTimer("total");
    current_frame_data_.cones_detected = cones;
    frames_.push_back(current_frame_data_);
}

void PerformanceProfiler::saveToJSON(const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        std::cerr << "Error: Unable to open JSON file for saving performance data: " << filepath << std::endl;
        return;
    }

    // Manual JSON serialization for maximum control and zero external dependencies
    out << "{\n";
    out << "  \"metadata\": {\n";
    out << "    \"algorithm\": \"" << algorithm_name_ << "\",\n";
    out << "    \"total_frames\": " << frames_.size() << "\n";
    out << "  },\n";
    
    out << "  \"raw_frames\": [\n";
    for (size_t i = 0; i < frames_.size(); ++i) {
        const auto& f = frames_[i];
        out << "    {\n";
        out << "      \"frame_id\": " << f.frame_id << ",\n";
        out << "      \"conversion_ms\": " << std::fixed << std::setprecision(3) << f.conversion_ms << ",\n";
        out << "      \"deskewing_ms\": " << std::fixed << std::setprecision(3) << f.deskewing_ms << ",\n";
        out << "      \"ground_removal_ms\": " << std::fixed << std::setprecision(3) << f.ground_removal_ms << ",\n";
        out << "      \"clustering_ms\": " << std::fixed << std::setprecision(3) << f.clustering_ms << ",\n";
        out << "      \"merging_ms\": " << std::fixed << std::setprecision(3) << f.merging_ms << ",\n";
        out << "      \"estimation_ms\": " << std::fixed << std::setprecision(3) << f.estimation_ms << ",\n";
        out << "      \"duplicate_ms\": " << std::fixed << std::setprecision(3) << f.duplicate_ms << ",\n";
        out << "      \"total_ms\": " << std::fixed << std::setprecision(3) << f.total_ms << ",\n";
        out << "      \"cones_detected\": " << f.cones_detected << "\n";
        out << "    }";
        if (i < frames_.size() - 1) {
            out << ",";
        }
        out << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    
    out.close();
}

} // namespace fs_perception
