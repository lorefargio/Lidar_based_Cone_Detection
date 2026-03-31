# Pipeline Architecture and High-Level Workflow

This document provides a high-level overview of the LiDAR perception pipeline, detailing the data flow from raw sensor input to semantic cone detections.

## System Overview

The pipeline is designed as a sequential execution model, optimized for deterministic latency and real-time control (20Hz target).

```mermaid
graph TD
    A[Raw PointCloud2] --> B[Pre-processing]
    B --> C[Deskewing]
    C --> D[Ground Removal]
    D --> E[Clustering]
    E --> F[Weighted Aggregation]
    F --> G[Cone Estimation]
    G --> H[Final Markers]

    subgraph "Phase 1: Stabilization"
        B -- "3.5cm Voxel Filter" --> C
    end

    subgraph "Phase 2: Grouping"
        E -- "12cm Grid Resolution" --> F
    end

    subgraph "Phase 3: Classification"
        F -- "Soft-Pass Hysteresis" --> G
    end
```

### Stage Descriptions

1.  **Pre-processing**: Rapid spatial truncation and early voxelization. The **3.5cm voxel** size is chosen to balance geometric fidelity (preserving the cone's shape) and cardinality management (reducing total points).
2.  **Deskewing**: High-frequency IMU integration (NLERP) to compensate for sensor motion during the sweep.
3.  **Ground Removal**: Binary segmentation of the environment into traversable surface and obstacle candidates (using Patchwork++ or Slope Analysis).
4.  **Clustering**: Spatial grouping of obstacle points into candidate clusters.
5.  **Weighted Aggregation**: Instead of standard NMS, positions are averaged across nearby candidates to ensure smooth, fluid motion in the output.
6.  **Cone Estimation**: Bayesian-like geometric validation using PCA-derived features (linearity, planarity, scattering) and dynamic thresholds with soft-pass logic.
