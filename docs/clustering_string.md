# String Clusterer: Ultra-Fast Linear Scan grouping

The `StringClusterer` is a linear-time grouping strategy that exploits the native point-ordering of modern LiDAR sensors. By processing points in their sequential "scan-line" order, it avoids all expensive spatial search structures (KD-Trees, grids).

## Abstract & Motivation
LiDAR drivers (e.g., Ouster, Velodyne) publish points in a deterministic order: by **ring** (vertical index) and then by **firing** (azimuthal index). This means that points $p_i$ and $p_{i-1}$ in the cloud are often neighbors in the real world. `StringClusterer` groups these consecutive points into "strings" or object segments in a single pass ($O(N)$).

## Mathematical Foundation
A string $\mathcal{S}$ is formed if consecutive points $p_i$ and $p_{i-1}$ satisfy both spatial and intensity constraints:
1. **Dynamic Distance Gate**:
   $$\text{dist}(p_i, p_{i-1}) < \tau(r) = \tau_{base} + \beta \cdot r$$
2. **Intensity Continuity**:
   $$|\text{intensity}_i - \text{intensity}_{i-1}| < I_{max}$$
The intensity check is crucial for detecting object boundaries where points are spatially close but have vastly different reflective properties (e.g., a matte cone in front of a shiny wall).

## Implementation Walkthrough

### 1. Sequential Scan logic
The algorithm is implemented as a single `for` loop over the point cloud.
```cpp
for (size_t i = 1; i < cloud->points.size(); ++i) {
    const auto& pt = cloud->points[i];
    const auto& prev = cloud->points[i-1];
    // ... connectivity check ...
}
```

### 2. Distance Scaling
The threshold $\tau(r)$ is updated point-wise to account for azimuthal separation:
```cpp
float range = std::sqrt(pt.x*pt.x + pt.y*pt.y);
float dynamic_threshold = config_.max_dist + (range * 0.02f);
```

### 3. Cluster Seal and Restart
When the connectivity check fails, the current `current_cluster` is "sealed" (sent to output if it meets size criteria) and a new candidate is started.

## Complexity Analysis
- **Time Complexity**: **$O(N)$**. The most efficient possible complexity, as each point is visited exactly once.
- **Space Complexity**: **$O(N)$** for the temporary storage of candidate clusters.

## Citations & Literature
- **Primary Method**: Bogoslavskyi, I., & Stachniss, C. (2016). *"Fast range image-based segmentation of sparse 3D Lidar scans."* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
- **Scan-line Concept**: Zermas, D., Izzat, I., & Papanikolopoulos, N. (2017). *"Fast Segmentation of 3D Point Clouds for Ground Vehicles."* IEEE International Conference on Robotics and Automation (ICRA).
