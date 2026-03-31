# DBSCAN: Density-Based Spatial Clustering with Hash-Grid Optimization

The `DBSCANClusterer` is a robust grouping strategy that can identify clusters of arbitrary shapes and filter out outlier noise. Our implementation optimizes the traditional $O(N \log N)$ algorithm to achieve near-linear performance using a **3D Flat Grid**.

## Abstract & Motivation
Density-Based Spatial Clustering of Applications with Noise (DBSCAN) is ideal for LiDAR because it doesn't require pre-specifying the number of clusters (unlike K-Means) and it inherently handles environmental noise. However, standard PCL-based DBSCAN uses KD-Trees for radius searches, which can be computationally expensive. Our implementation uses a discretized 3D volume for $O(1)$ neighbor lookup.

## Mathematical Foundation
Given a point $p$ and radius $\epsilon$, the neighborhood $N_{\epsilon}(p)$ is:
$$N_{\epsilon}(p) = \{q \in \mathcal{P} \mid \text{dist}(p, q) \leq \epsilon\}$$
- **Core Point**: A point $p$ where $|N_{\epsilon}(p)| \geq MinPts$.
- **Density-Reachable**: A point $q$ is density-reachable from $p$ if there's a path of core points connecting them.
- **Cluster**: A maximal set of density-reachable points.

## Implementation Walkthrough

### 1. Hash-Killer: 3D Flat Grid
To avoid the overhead of `std::unordered_map` (hashing, collisions), we pre-allocate a 1D vector representing a constrained 3D volume (+/- 30m XY, +/- 2m Z).
```cpp
const int dim_xy = static_cast<int>(2.0f * range_xy * inv_eps) + 1;
const int dim_z = static_cast<int>(2.0f * range_z * inv_eps) + 1;
std::vector<std::vector<int>> grid(dim_xy * dim_xy * dim_z);
```
Mapping 3D coordinates $(x, y, z)$ to a 1D index is done using:
$$\text{Index} = (gx \cdot \text{dim}_{xy} \cdot \text{dim}_z) + (gy \cdot \text{dim}_z) + gz$$

### 2. Neighborhood Search Optimization
The search radius $\epsilon$ is used as the grid's cell size. Any point within distance $\epsilon$ of $p$ is guaranteed to be either in $p$'s cell or one of its **26 adjacent cells**.
```cpp
for (int x = gx - 1; x <= gx + 1; ++x) {
    for (int y = gy - 1; y <= gy + 1; ++y) {
        for (int z = gz - 1; z <= gz + 1; ++z) {
            // Check 27 cells...
        }
    }
}
```
This reduces radius search from $O(\log N)$ (tree) to $O(1)$ (grid lookup).

### 3. Cluster Expansion logic
- **Labels**: `-1` for unvisited, `-2` for noise, `0..K` for cluster ID.
- **Queue**: A `seed_queue` is used to expand from each core point discovered.

## Complexity Analysis
- **Time Complexity**: $O(N)$ for grid population + $O(N \cdot |N_{\epsilon}|)$ for expansion. In practice, $|N_{\epsilon}|$ is small and constant for cones, leading to **near-linear** performance.
- **Space Complexity**: $O(N + \text{GridVolume})$. The 3D grid size is the primary memory overhead.

## Citations & Literature
- **Original Algorithm**: Ester, M., Kriegel, H. P., Sander, J., & Xu, X. (1996). *"A density-based algorithm for discovering clusters in large spatial databases with noise."* KDD-96 Proceedings.
- **Grid-based Acceleration**: Gunawan, A. (2013). *"A faster algorithm for DBSCAN."* This approach is a variant of "Grid-DBSCAN."
