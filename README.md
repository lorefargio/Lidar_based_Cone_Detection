# High-Performance LiDAR Perception for Autonomous Racing

## Abstract
This repository implements a modular, high-performance perception system designed for track-boundary localization (traffic cones) in Formula Student Driverless environments. The system is engineered for **Humble ROS 2** and optimized for a 20Hz update rate. The core architecture prioritizes deterministic execution, O(1) spatial indexing, and SIMD-accelerated transformations to facilitate robust control in high-speed maneuvers.

## System Architecture
The pipeline is structured as a sequential execution model with the following stages:

1.  **Preprocessing & Motion Compensation**: Implements non-linear temporal interpolation (Nlerp) for LiDAR deskewing and mandatory early voxelization to ensure cardinality bounds for downstream stages.
2.  **Adaptive Ground Segmentation**: Employs **Patchwork++** (Lee et al., 2022) and Radial Slope Analysis to isolate non-ground obstacles while preserving base-points at high radial ranges.
3.  **Spatial Grouping (Clustering)**: Utilizes Optimized DBSCAN and Connected Components, accelerated via **Flat 3D Grid Indexing** to eliminate KD-Tree reconstruction and hashing overhead.
4.  **Geometric Estimation & Classification**: Performs covariance-based shape analysis (PCA) and rule-based validation with dynamic distance-aware thresholds.

## Technical Documentation
For detailed theoretical derivation and implementation justifications, refer to the following documents:

*   **[LiDAR Deskewing](docs/deskewing.md)**: Motion compensation and temporal synchronization.
*   **[Ground Segmentation](docs/filtering.md)**: Patchwork++ (CZM, RNR, R-VPF, A-GLE) and Slope Analysis.
*   **[Spatial Clustering](docs/clustering.md)**: O(1) Hash-Grid DBSCAN and Adaptive Density models.
*   **[Geometric Estimation](docs/estimation.md)**: Covariance analysis and voxel-aware classification.
*   **[System Configuration](docs/parameters.md)**: Comprehensive guide to runtime parameters and YAML tuning.
*   **[Performance Benchmarking](docs/benchmarking.md)**: Methodology for P99 and PDF latency analysis.

## Core Optimization Principles
*   **Memory Safety**: Persistent buffer reuse and zero-copy publishing (move semantics) to minimize heap fragmentation.
*   **Algorithmic Determinism**: Transition from $O(N \log N)$ to $O(N)$ via flat-grid spatial partitioning.
*   **Real-Time QoS**: Utilization of `SensorDataQoS` and decimation-based debug publishing to ensure bandwidth availability for safety-critical topics.

## Build and Execution
```bash
# Optimized build for release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Execution
ros2 launch fs_lidar_perception foxglove.launch.py ground_remover_type:=patchworkpp
```
