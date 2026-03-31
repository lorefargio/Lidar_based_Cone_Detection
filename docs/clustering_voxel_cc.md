# Voxel Connected Components: 3D Hashing-Based Grouping

The `VoxelConnectedComponents` clusterer is a robust 3D grouping strategy that strikes a balance between speed and geometric fidelity. It uses a sparse grid (hash-map) to identify 3D object candidates based on voxel connectivity.

## Abstract & Motivation
Unlike the 2D Grid Clusterer, `VoxelConnectedComponents` operates in a true 3D space, which allows it to distinguish between objects that overlap in 2D but are separated vertically (e.g., a cone under a bridge or overhanging vegetation). By using a sparse grid instead of a dense flat grid, it efficiently handles large, sparse outdoor environments.

## Mathematical Foundation
The world is discretized into cubic voxels $V$ of side length $\lambda$. Each point $p_i$ is hashed to a voxel $V_{x, y, z}$:
$$x = \lfloor x_i / \lambda \rfloor, \quad y = \lfloor y_i / \lambda \rfloor, \quad z = \lfloor z_i / \lambda \rfloor$$
The algorithm uses a **26-neighbor connectivity** model. Two voxels $V_A$ and $V_B$ are connected if they share at least one vertex:
$$\max(|x_A - x_B|, |y_A - y_B|, |z_A - z_B|) \leq 1$$

## Implementation Walkthrough

### 1. Voxelization (Phase 1)
Points are inserted into an `std::unordered_map` where the key is a `VoxelKey` (3 integer coordinates). This is an **$O(N)$** process.
```cpp
for (size_t i = 0; i < cloud->size(); ++i) {
    VoxelKey k = {
        static_cast<int>(std::floor(pt.x * inv_voxel_size_)),
        static_cast<int>(std::floor(pt.y * inv_voxel_size_)),
        static_cast<int>(std::floor(pt.z * inv_voxel_size_))
    };
    voxel_grid[k].push_back(i);
}
```

### 2. 3D Connected Components via BFS (Phase 2)
The algorithm iterates over all occupied voxels. For each unvisited voxel, it initiates a BFS search across all 26 possible neighbors.
- **Queue**: `std::queue<VoxelKey> queue`.
- **Connectivity Kernel**: Precomputed offsets `dx`, `dy`, `dz` for all 27 permutations (excluding self).

### 3. Hashing and Hashing Performance
The `VoxelKeyHasher` uses XOR-based bit-shifting to combine the three integer coordinates into a single hash value. This is a common technique to minimize collisions in sparse spatial structures.
```cpp
return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
```

## Complexity Analysis
- **Time Complexity**: **$O(N)$** for hashing and bining. BFS is **$O(V)$**, where $V$ is the number of occupied voxels. Since $V \leq N$, the total complexity is $O(N)$. However, the constant factor of `std::unordered_map` is higher than a flat grid.
- **Space Complexity**: **$O(N)$** for storing point indices and the hash map.

## Citations & Literature
- **Primary Method**: Standard computer vision technique; see: He, L., Chao, Y., & Suzuki, K. (2017). *"Connected-component labeling: A review."* for detailed analysis of connectivity models.
- **Volumetric Representation**: Curless, B., & Levoy, M. (1996). *"A volumetric method for building complex models from range images."* SIGGRAPH Proceedings.
