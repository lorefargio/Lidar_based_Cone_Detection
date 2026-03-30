# Spatial Clustering and Object Aggregation Strategies

## Theoretical Overview
Spatial clustering is the process of partitioning the obstacle subset $\mathcal{P}_o$ into $K$ disjoint clusters $\{\mathcal{C}_1, \dots, \mathcal{C}_K\}$ based on spatial proximity. In high-speed autonomous racing, the clustering stage must handle variable point density, sensor noise, and strict latency bounds.

### 1. From $O(N \log N)$ to $O(N)$: The Spatial Hash Grid
Traditional clustering algorithms (e.g., standard Euclidean or DBSCAN) rely on tree-based structures (KD-Trees or Octrees) for nearest neighbor queries. While effective for general applications, tree reconstruction at every frame introduces significant overhead and non-deterministic cache access patterns.

#### Flat Grid Optimization ("Hash-Killer")
To achieve $O(N)$ complexity, we implement a **Pre-allocated 3D Spatial Grid**. The operational volume (e.g., $\pm 30m$ in $X, Y$) is discretized into a flat 1D vector. 
*   **Coordinate Mapping**: The mapping from 3D coordinates $(x, y, z)$ to a linear index $I$ is performed via integer floor division and offset addition:
    $$I = \lfloor \frac{x+R_x}{\epsilon} \rfloor \cdot D_y \cdot D_z + \lfloor \frac{y+R_y}{\epsilon} \rfloor \cdot D_z + \lfloor \frac{z+R_z}{\epsilon} \rfloor$$
*   **Justification**: This eliminates the computational cost of hashing (needed for `std::unordered_map`) and ensures that memory access is contiguous and cache-efficient. Neighbor search is reduced to a constant-time check of the 26 adjacent cells.

### 2. Adaptive Density-Aware DBSCAN (A-DBSCAN)
Point density in LiDAR scans follows an inverse-square relationship with radial range $r$. Standard DBSCAN with a fixed $\epsilon$ and $MinPts$ fails to segment distant objects (under-segmentation) or merges close-proximity objects (over-segmentation).

#### Exponential Decay MinPts Model
We propose a distance-adaptive model where the density requirement $\kappa$ and search radius $\epsilon$ are functions of $r$:
1.  **Search Radius**: $\epsilon(r) = \epsilon_{base} + \alpha r$, compensating for beam divergence.
2.  **Cardinality Threshold**: $\kappa(r) = \max(\kappa_{min}, \lfloor \kappa_{base} \cdot e^{-\gamma r} \rfloor)$, where $\gamma$ is the decay constant calibrated to the sensor's vertical resolution.
*   **Result**: This allows the system to identify far-field cones consisting of only 3-5 points while remaining robust against noise in the near-field.

### 3. Euclidean Cluster Extraction with Voxel-Decimation
For cases where high precision is required, the standard Euclidean extraction is utilized but optimized with **Internal Voxel Decimation**.
*   **Optimization**: If $| \mathcal{P}_o | > \text{Threshold}$, a 4cm voxel filter is applied internally. This maintains the geometric topology of the clusters while reducing the tree-building complexity $O(N_{vox} \log N_{vox})$ where $N_{vox} \ll N$.

## Comparative Performance Analysis
| Metric | Grid Clusterer | Optimized DBSCAN | Euclidean (PCL) |
| :--- | :--- | :--- | :--- |
| **Complexity** | $O(N)$ | $O(N)$ (Hash-Grid) | $O(N \log N)$ |
| **Noise Robustness** | Low | High | Medium |
| **Vertical Overlap** | No (2D) | Yes (3D) | Yes (3D) |


## Summary of Implementation Choices
The transition to **Flat Grid Indexing** is the primary driver for our pipeline's real-time safety. By sacrificing the infinite flexibility of a hash map for a bounded, pre-allocated volume, we achieve the deterministic execution times required for sub-20ms perception.
