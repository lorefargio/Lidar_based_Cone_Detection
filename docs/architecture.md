# Pipeline Architecture and High-Level Workflow

This document provides a high-level overview of the LiDAR perception pipeline, detailing the data flow from raw sensor input to semantic cone detections.

## System Overview

The pipeline is designed as a sequential execution model, optimized for deterministic latency and real-time control (20Hz target).

```mermaid
graph TD
    A[Raw PointCloud2] --> B[Pre-processing / Truncation]
    B --> C[Deskewing]
    C --> V[Voxelization]
    V --> D[Ground Removal]
    D --> E[Clustering]
    E --> F[Cluster Merging]
    F --> G[Cone Estimation]
    G --> H[Spatial Aggregation]
    H --> I[Final Markers]

    subgraph "Phase 1: Stabilization & Downsampling"
        B --> C
        C -- "2.0cm Voxel Filter" --> V
    end

    subgraph "Phase 2: Grouping"
        E -- "Gated Proximity" --> F
    end

    subgraph "Phase 3: Classification"
        G --> H
    end
```

### Stage Descriptions

1.  **Pre-processing / Truncation**: Rapid point-wise spatial truncation and blind-spot removal to clean raw sensor data safely before motion compensation. Because this is point-wise, it does not aggregate or blur points.
2.  **Deskewing**: High-frequency IMU integration (NLERP) to compensate for sensor motion during the sweep. This must run on raw points before voxelization to prevent motion distortion blurring.
3.  **Voxelization**: Downsampling the deskewed cloud using a 2.0cm voxel grid to leverage the high resolution of the 40-channel LiDAR (0.33° vertical) while reducing total points.
4.  **Ground Removal**: Binary segmentation of the environment into traversable surface and obstacle candidates (using Patchwork++ or Slope Analysis).
5.  **Clustering**: Spatial grouping of obstacle points into candidate clusters.
6.  **Cluster Merging**: Geometric unification of fragmented clusters. Candidates within **0.25m** are merged to form a single volumetric object, improving detection stability for sparse returns.
7.  **Cone Estimation**: Bayesian-like geometric validation using PCA-derived features (linearity, planarity, scattering) and dynamic thresholds with soft-pass logic.
8.  **Spatial Aggregation**: Position averaging across nearby candidates (unweighted NMS equivalent) to ensure stable output.
9.  **Final Markers**: Asynchronous publication of visualization markers and semantic point clouds for fusion.
