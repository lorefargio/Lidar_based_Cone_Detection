# Spatial Clustering and Object Aggregation Strategies

Spatial clustering partitions the ground-removed obstacle point cloud $`\mathcal{P}_o`$ into $`K`$ disjoint candidate objects $`\{\mathcal{C}_1, \dots, \mathcal{C}_K\}`$ corresponding to physical elements (e.g. traffic cones) in the environment.

---

## 1. Algorithmic Comparison Matrix

The table below summarizes the key trade-offs between the 6 clustering algorithms implemented in this package:

| Algorithm | Complexity | Dimensionality | Best Use Case | Primary Hyperparameters |
| :--- | :--- | :--- | :--- | :--- |
| **Grid BFS** | $\mathcal{O}(N)$ | 2D | Flat track racing, minimal Z-overlap, ultra-low latency. | `grid_resolution` (0.12m) |
| **DBSCAN** | $\approx \mathcal{O}(N)$ | 3D | General purpose, noise-tolerant. | `dbscan_eps` (0.30m), `dbscan_min_pts` (3) |
| **HDBSCAN** | $\mathcal{O}(N^2)$ | 3D | High-precision varying density, lidar-aware stability. | `hdbscan_min_pts` (5), `hdbscan_alpha` (0.02) |
| **Euclidean** | $\mathcal{O}(N \log N)$ | 3D | High-precision standard setups (via PCL KD-Tree). | `euclidean_tolerance` (0.35m) |
| **Depth Clusterer** | $\mathcal{O}(N)$ | 3D-Topological | Sensor-grid BFS, ultra-low latency, scale-invariant. | `depth_theta_thr` (10.0°), `depth_num_rings` (32) |
| **Voxel CC** | $\mathcal{O}(N)$ | 3D | Sparse data requiring Z-axis separation. | `voxel_grid_size` (0.02m) |

---

## 2. Technical Deep-Dives

### 2.1 Grid BFS (2D Connected Components)
The `GridClusterer` projects the 3D obstacle points onto a flat 2D grid $`\mathcal{G}`$ with cell size $`\delta`$.
*   **Mathematical Model**:
    Each point $`p_i = (x_i, y_i, z_i)`$ is mapped to cell coordinates $`(gx_i, gy_i)`$:
    $$
    gx_i = \lfloor x_i / \delta \rfloor, \quad gy_i = \lfloor y_i / \delta \rfloor
    $$
    An **8-connectivity connectivity model** groups cells. Two occupied cells $`A`$ and $`B`$ belong to the same component if:
    $$
    \max(|gx_A - gx_B|, |gy_A - gy_B|) \leq 1
    $$
*   **Implementation Specifics**:
    Implemented using a flat `std::vector` to maximize cache localization. To prevent $\mathcal{O}(\text{GridSize})$ initialization overhead, it tracks `occupied_indices` in a dynamic array, ensuring that only occupied cells are cleared between frames. This maintains a strict $\mathcal{O}(N)$ runtime.

### 2.2 Hash-Grid DBSCAN
Standard DBSCAN requires $\mathcal{O}(N \log N)$ KD-Tree searches. The `DBSCANClusterer` optimizes this by binning points in a pre-allocated 3D sparse spatial hash map, reducing regional query bounds to $\mathcal{O}(1)$.
*   **Mathematical Model**:
    For a given point $`p`$, a neighborhood search is defined as:
    $$
    N_{\epsilon}(p) = \{q \in \mathcal{P}_o \mid \|p - q\| \leq \epsilon\}
    $$
    Points are classified as **Core Points** if $`|N_{\epsilon}(p)| \geq \text{MinPts}`$. Non-core points within the neighborhood of a core point are boundary points; others are noise.
*   **Implementation Specifics**:
    Instead of full radius searches, the neighbor search looks only inside the point's current 3D voxel cell and its 26 immediate spatial neighbors. This avoids construction and balancing overhead of KD-Trees.

### 2.3 HDBSCAN (Hierarchical DBSCAN)
HDBSCAN generalizes DBSCAN by converting density-based grouping into a hierarchical tree structure, extracting clusters according to their stability across varying distance metrics.
*   **Mathematical Model**:
    Defines the **Mutual Reachability Distance** between points $`p`$ and $`q`$:
    $$
    d_{\text{mred}}(p, q) = \max(d_{\text{core}}(p), d_{\text{core}}(q), d(p, q))
    $$
    where $`d_{\text{core}}(p)`$ is the Euclidean distance to its $`k`$-th nearest neighbor. A Minimum Spanning Tree is constructed based on $`d_{\text{mred}}`$ and pruned to produce a hierarchical tree representing cluster stability.
*   **Implementation Specifics**:
    A Lidar-aware implementation that dynamically scales $`d_{\text{core}}`$ with range to accommodate the natural beam divergence and drop in density at long distances.

### 2.4 Euclidean Clustering (PCL KD-Tree)
A standard 3D spatial partitioning algorithm.
*   **Mathematical Model**:
    Extracts clusters where the spatial distance between any point in the cluster and another is less than `euclidean_tolerance`:
    $$
    \forall p \in \mathcal{C}, \exists q \in \mathcal{C} \mid \|p - q\| \leq d_{\text{tol}}
    $$
*   **Implementation Specifics**:
    Leverages `pcl::EuclideanClusterExtraction` backed by a `pcl::search::KdTree`. It represents the baseline for precision but suffers from the $\mathcal{O}(N \log N)$ tree balancing cost.

### 2.5 Depth Clustering (Range Image BFS)
Projects the 3D point cloud into a 2D spherical depth image where rows correspond to the LiDAR's vertical channels (laser rings) and columns correspond to horizontal angle bins.
*   **Mathematical Model**:
    Checks the angle $`\beta`$ between two adjacent points in the range image:
    $$
    \beta = \arctan2\left(d_2 \sin \Delta\theta, d_1 - d_2 \cos \Delta\theta\right)
    $$
    where $`d_1, d_2`$ are the depth values, and $`\Delta\theta`$ is the angular resolution between beams. If $`\beta`$ exceeds a threshold, the points belong to the same physical object.
*   **Implementation Specifics**:
    Executes a fast BFS directly on the spherical image grid. This is highly scale-invariant and extremely fast because it bypasses all 3D spatial index construction.

### 2.6 Voxel Connected Components
Bins the obstacles in a 3D voxel grid.
*   **Mathematical Model**:
    Groups voxels based on 26-connectivity in a 3D grid.
*   **Implementation Specifics**:
    Similar to 2D Grid BFS but preserves the height ($`Z`$) axis. Helpful in scenarios where objects might overlap on the ground plane but have vertical gaps.

---

## 3. Pipeline Tuning and Optimization
*   **Resolution Tuning**: Lowering `grid_resolution` from **20cm to 12cm** is mandatory to prevent spatial aliasing when detecting regulation Formula Student cones (which have a base width of 22.8cm).
*   **Minimum Cluster Size**: Configured to **2 points** to maximize detection recall at the vehicle's telemetry limits (25m), where sparse returns only hit the cone with 2 or 3 laser points.
*   **Volumetric Merging**: Fragmented clusters within **0.25m** are merged to form a single volumetric candidate prior to PCA estimation.
