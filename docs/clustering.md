# Spatial Clustering and Object Aggregation Strategies

Spatial clustering partitions the obstacle cloud $\mathcal{P}_o$ into $K$ disjoint candidate objects $\{\mathcal{C}_1, \dots, \mathcal{C}_K\}$.

## Strategy Comparison Matrix

The following table provides a high-level comparison of the available clustering strategies:

| Algorithm | Complexity | Dimensionality | Best Use Case | Detail Page |
| :--- | :--- | :--- | :--- | :--- |
| **Grid BFS** | $O(N)$ | 2D | Flat track racing, minimal Z-overlap. | [Read more](./clustering_grid.md) |
| **DBSCAN** | $\approx O(N)$ | 3D | General purpose, noise-tolerant. | [Read more](./clustering_dbscan.md) |
| **Adaptive DBSCAN** | $O(N)$ | 3D | High-speed racing, long-range detection. | [Read more](./clustering_adaptive_dbscan.md) |
| **Euclidean (KD-Tree)** | $O(N \log N)$ | 3D | High precision, standard PCL wrapper. | [Read more](./clustering_euclidean.md) |
| **String (Scan-line)** | $O(N)$ | 3D-ish | Ultra-low latency, driver-level sorting. | [Read more](./clustering_string.md) |
| **Voxel CC** | $O(N)$ | 3D | Sparse data, Z-separation needed. | [Read more](./clustering_voxel_cc.md) |

## Clustering Algorithm Summaries

### 1. Grid Clusterer (BFS-Connected Components)
Optimized for 2D flat track scenarios. Points are projected onto a 2D grid where neighboring cells are grouped using Breadth-First Search (BFS).

```mermaid
graph LR
    A[Obstacle Points] --> B[2D Projection]
    B --> C[12cm Flat Grid Population]
    C --> D{BFS Search}
    D -- "8-Connectivity" --> E[Cluster Extraction]
    E --> F[Size Filtering]
```
- **Complexity**: Strictly $O(N)$ due to the pre-allocated flat grid.
- **Limitation**: Ignores Z-axis separation.
- **Detailed Docs**: [Grid Clusterer Technical Deep-Dive](./clustering_grid.md)

### 2. Hash-Grid Optimized DBSCAN
Combines density-based robustness with deterministic $O(1)$ neighbor lookup.

```mermaid
graph TD
    A[Pre-allocated 3D Grid] --> B[Map Points to Cells]
    B --> C[Core Point Discovery]
    C -- "Check 27 neighboring cells" --> D{Density Check}
    D -- ">= MinPts" --> E[Expand Cluster]
    D -- "< MinPts" --> F[Noise/Outlier]
```
- **Performance**: Eliminates KD-Tree rebuild overhead, ensuring P99 stability.
- **Detailed Docs**: [DBSCAN Technical Deep-Dive](./clustering_dbscan.md)

### 3. Adaptive DBSCAN
Extends DBSCAN with a distance-scaling parameter model to compensate for LiDAR beam divergence at high range.
- **Detailed Docs**: [Adaptive DBSCAN Technical Deep-Dive](./clustering_adaptive_dbscan.md)

### 4. String Clusterer (Linear Scan)
Utilizes the sequential nature of LiDAR sweeps to group points in a single pass.

```mermaid
graph LR
    A[Ring-Ordered Cloud] --> B[Scan Consecutive Points]
    B --> C{Distance Gate < 0.3m}
    C -- "Yes" --> D[Append to Current String]
    C -- "No" --> E[Seal and Filter String]
    E --> F[New Object Candidate]
```
- **Prerequisite**: Requires driver-level ring sorting.
- **Detailed Docs**: [String Clusterer Technical Deep-Dive](./clustering_string.md)

## Summary of Optimization Changes
- **Grid Resolution**: Lowered from **20cm to 12cm** to minimize spatial aliasing for standard cones (22.8cm width).
- **Min Cluster Size**: Relaxed to **2 points** to improve detection at extreme ranges where point density is minimal.
