# Performance Benchmarking and Metric Analysis

## Methodology
To validate the real-time suitability of the perception pipeline, we employ a multi-variate benchmarking framework. Unlike standard testing, our focus is on **Deterministic Latency** and **P99 Stability**.

### Core Metrics

#### 1. P99 (99th Percentile) Latency
In autonomous racing, the "Average Latency" is a misleading metric. A single frame exceeding the 50ms budget can cause a loss of control at high speeds. We prioritize P99 analysis to ensure that 99% of processed frames respect the real-time constraints.

#### 2. PDF (Probability Density Function) Analysis
We use Kernel Density Estimation (KDE) to visualize the jitter distribution. A narrow "peak" indicates a stable, predictable system, while multiple peaks or heavy tails suggest cache misses, OS scheduling interference, or $O(N \log N)$ complexity bottlenecks.

#### 3. Faceted Distribution Grid
Our plotting utility generates separate distribution charts for each algorithm configuration. This allows for a granular comparison of how different ground segmentation and clustering strategies impact the end-to-end P99.

## Experimental Setup
1.  **Environment**: CPU-isolated threads, Release build with LTO.
2.  **Dataset**: High-dynamic rosbag ($v > 15m/s$) with diverse cone distributions.
3.  **Validation**: Synchronized visualization in Foxglove to ensure zero-lag between original cloud and detected markers.

## Result Interpretation
*   **Bar Charts (Breakdown)**: Identifies the bottleneck phase (e.g., if DBSCAN dominates the total time).
*   **Faceted Stability**: Monitors "Cones Detected" over time. A stable algorithm should show minimal flickering in count between consecutive frames.
