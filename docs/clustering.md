# Spatial Clustering and Object Aggregation Strategies

Spatial clustering partitions the obstacle cloud $\mathcal{P}_o$ into $K$ disjoint candidate objects $\{\mathcal{C}_1, \dots, \mathcal{C}_K\}$.

## Clustering Algorithm Deep-Dive

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

### 3. String Clusterer (Linear Scan)
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

## Summary of Optimization Changes
- **Grid Resolution**: Lowered from **20cm to 12cm** to minimize spatial aliasing for standard cones (22.8cm width).
- **Min Cluster Size**: Relaxed to **2 points** to improve detection at extreme ranges where point density is minimal.
