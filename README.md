# High-Performance LiDAR Perception for Autonomous Racing

## Abstract
This repository implements a modular, high-performance perception system designed for localization (traffic cones) in autonomous environments. The system is engineered for **Humble ROS 2** and optimized for a 20Hz update rate.

## System Architecture
The pipeline is optimized for **Stability** and **Fluidity** through a three-stage refinement model:

1.  **Geometric Pre-processing**: Uses a **3.5cm early voxel grid** to maintain high shape fidelity for small objects like cones.
2.  **Modular Segmentation**: Employs adaptive ground removal and high-resolution **(12cm grid)** clustering.
3.  **Hysteresis Estimation**: Performs rule-based classification with **Soft-Pass logic** and **Weighted Spatial Aggregation** for jitter-free output.

## Technical Documentation
For detailed theoretical derivation and implementation justifications, refer to:

*   **[Pipeline Architecture](docs/architecture.md)**: High-level workflow and Mermaid diagrams.
*   **[Ground Segmentation](docs/filtering.md)**: Patchwork++ and Slope Analysis.
*   **[Spatial Clustering](docs/clustering.md)**: O(1) Grid, DBSCAN, and String algorithms explained.
*   **[Geometric Estimation](docs/estimation.md)**: PCA, Soft-Pass logic, and Weighted Aggregation.
*   **[System Configuration](docs/parameters.md)**: Comprehensive guide to runtime parameters.

## Core Optimization Principles
*   **Temporal Stability**: Weighted centroid aggregation for smooth coordinate reporting.
*   **Rigorous Timestamp Preservation**: All output messages (/perception/cones, /perception/cone_points) inherit the exact `header.stamp` of the hardware-triggered `/lidar_points` message, ensuring downstream fusion nodes maintain perfect synchronization regardless of processing latency.
*   **Zero-Copy IPC**: Optimized for Inter-Process Communication using `std::unique_ptr` and ROS 2 middleware bypasses, significantly reducing deserialization overhead for high-bandwidth point cloud streams.
*   **Algorithmic Determinism**: Transition from $O(N \log N)$ to $O(N)$ via flat-grid spatial partitioning.
*   **Hysteresis Logic**: Multi-threshold validation to prevent detection flickering.

## Build and Execution
```bash
# Optimized build for release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Execution
ros2 launch lidar_perception foxglove.launch.py ground_remover_type:=patchworkpp
```
