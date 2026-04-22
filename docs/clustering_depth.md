# Depth-Clustering: Range-Image BFS Segmentation

The `DepthClusterer` is a State-of-the-Art (SOTA) algorithm that evolves the 1D String Clusterer into a 2D topological grouping strategy. It is designed for ultra-low latency and scale-invariant precision.

## Abstract & Motivation
Traditional 3D clustering (like DBSCAN) or 1D scan-line grouping (String Clusterer) often fail to capture the full 3D structure of objects without external merging steps. `DepthClusterer` solves this by projecting the point cloud into a **Range Image** and performing a 4-connected BFS segmentation directly on the sensor's topology.

## Mathematical Foundation
The algorithm uses the **Angle $\beta$** criterion to determine connectivity between two points $p_1$ and $p_2$ with distances $d_1$ and $d_2$:
$$\beta = \arctan2(d_{min} \cdot \sin \alpha, d_{max} - d_{min} \cdot \cos \alpha)$$
where $\alpha$ is the angular resolution between the rays.
- If $\beta > \theta_{threshold}$, the points belong to the same surface.
- This criterion is **independent of distance**, making it equally effective at 2m and 25m.

## Implementation Walkthrough

### 1. Range Image Generation
The point cloud is mapped to a 2D matrix where:
- **Rows**: Corresponding to the laser `ring`.
- **Columns**: Mapped from the azimuthal angle $\phi = \text{atan2}(y, x)$.
```cpp
int row = pt.ring;
float azimuth = std::atan2(pt.y, pt.x);
int col = static_cast<int>((azimuth + M_PI) / (2.0 * M_PI) * config_.num_cols) % config_.num_cols;
```

### 2. BFS Segmentation
A queue-based Breadth-First Search (BFS) traverses the image. For each occupied cell, it checks its 4 neighbors (Up, Down, Left, Right).
- **Horizontal Wraparound**: The columns are treated as a cylinder (`next_c % num_cols`) to handle the $360^\circ$ sweep continuity.
- **Topological Continuity**: Since neighbors in the image are neighbors in the real world, the algorithm captures the entire 3D volume of a cono in a single pass.

## Complexity Analysis
- **Time Complexity**: **$O(N)$**. Each point is visited exactly once during image population and at most once during BFS.
- **Space Complexity**: **$O(N + \text{ImageSize})$**. Requires a grid to store point indices.

## Citations & Literature
- **Primary Method**: Bogoslavskyi, I., & Stachniss, C. (2016). *"Fast range image-based segmentation of sparse 3D Lidar scans."* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
- **Angular Resolution**: Zermas, D., Izzat, I., & Papanikolopoulos, N. (2017). *"Fast Segmentation of 3D Point Clouds for Ground Vehicles."* IEEE International Conference on Robotics and Automation (ICRA).
