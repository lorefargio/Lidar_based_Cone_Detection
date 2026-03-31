# Grid Clusterer: 2D BFS-Connected Components

The `GridClusterer` is the most computationally efficient grouping strategy in the pipeline, achieving strictly **$O(N)$** time complexity. It is specifically designed for high-frequency perception on flat surfaces (e.g., racing tracks) where vertical separation is not a primary discriminator for object identification.

## Abstract & Motivation
In autonomous racing, the perception system must operate within tight latency bounds (typically $<50ms$ end-to-end). Traditional KD-Tree-based clustering ($O(N \log N)$) can become a bottleneck when processing dense point clouds. The `GridClusterer` avoids this by projecting the 3D world into a 2D sparse grid, transforming the clustering problem into a "Connected Components" problem in a discrete coordinate space.

## Mathematical Foundation
The algorithm discretizes the $\mathbb{R}^2$ plane into a grid $\mathcal{G}$ with resolution $\delta$. Each point $p_i = (x_i, y_i, z_i)$ is mapped to a grid cell $C_{gx, gy}$:
$$gx = \lfloor x_i / \delta \rfloor, \quad gy = \lfloor y_i / \delta \rfloor$$
The connectivity $\mathcal{K}$ is defined using an **8-neighbor connectivity** model. Two cells $C_A$ and $C_B$ are connected if:
$$\max(|gx_A - gx_B|, |gy_A - gy_B|) \leq 1$$

## Implementation Walkthrough

### 1. Spatial Discretization (Phase 1)
The grid is implemented as a flat `std::vector<Cell>` to ensure cache-friendly sequential access.
```cpp
// Grid dimension calculation ensuring coverage of max_range
grid_dim_ = static_cast<int>(std::ceil(2.0f * max_range_ / grid_res_)) + 2;
grid_.resize(grid_dim_ * grid_dim_);
```
Points are binned into cells. Crucially, the implementation tracks `occupied_indices` to optimize the subsequent search and reset phases.
```cpp
if (grid_[idx].point_indices.empty()) {
    occupied_indices.push_back(idx);
    grid_[idx].id = -1; // Reset state for current frame
}
grid_[idx].point_indices.push_back(i);
```

### 2. Connected Components via BFS (Phase 2)
The algorithm iterates only over `occupied_indices`. For each unvisited occupied cell, it initiates a Breadth-First Search (BFS) to find all reachable occupied neighbors.
- **Queue-based expansion**: Uses `std::queue<int> q` for FIFO traversal.
- **8-Connectivity Kernel**: Checks the 8 surrounding cells using precomputed offsets `dx` and `dy`.

### 3. Selective Reset (Phase 3)
A critical optimization is the "Selective Reset." Instead of clearing the entire `grid_` vector (which could have thousands of cells), only the cells in `occupied_indices` are cleared. This keeps the cleanup cost proportional to the number of points, not the grid volume.

## Complexity Analysis
- **Time Complexity**: $O(N)$ for binning, $O(N_{occupied})$ for BFS, $O(N_{occupied})$ for reset. Total: **$O(N)$**.
- **Space Complexity**: $O(N + \text{GridSize})$ for storing point indices and the flat grid structure.

## Citations & Literature
- **Primary Method**: Rosenfeld, A., & Pfaltz, J. L. (1966). *"Sequential operations in digital picture processing."* Journal of the ACM (JACM), 13(4), 471-494.
- **Grid Optimization**: Commonly used in high-speed robotics; see "Occupancy Grid Maps" in Thrun, S. (2002). *"Probabilistic Robotics."*
