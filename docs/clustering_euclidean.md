# Euclidean Clusterer: KD-Tree Based Grouping

The `EuclideanClusterer` is the standard PCL implementation wrapper, providing reliable spatial grouping using a $k$-dimensional tree (KD-Tree) for efficient neighbor searches.

## Abstract & Motivation
Euclidean clustering is the most common grouping strategy in point cloud processing. It's conceptually simple: points within a fixed Euclidean distance $\tau$ are part of the same cluster. This method is highly effective for well-separated objects (like cones on a track). While it is $O(N \log N)$, it provides high precision and is the industry-standard baseline.

## Mathematical Foundation
A cluster $\mathcal{C}$ is a set of points such that for any $p_i \in \mathcal{C}$, there exists a $p_j \in \mathcal{C}$ where:
$$\text{dist}(p_i, p_j) < \tau$$
where $\tau$ is the `cluster_tolerance`.

## Implementation Walkthrough

### 1. Internal VoxelGrid Optimization
To maintain performance on dense clouds, our implementation includes an internal downsampling step:
```cpp
if (cloud->size() > 5000) {
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(0.04f, 0.04f, 0.04f); // 4cm
    vg.filter(*filtered_cloud);
}
```
This ensures the KD-Tree construction and subsequent search calls remain within real-time budgets even in feature-rich environments.

### 2. KD-Tree Construction
The KD-Tree is rebuilt for each frame. This structure partitions the space hierarchically, enabling radius searches in $O(\log N)$ time on average.
```cpp
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
tree->setInputCloud(processed_cloud);
```

### 3. PCL Euclidean Extraction
The implementation leverages the mature `pcl::EuclideanClusterExtraction` class, which handles the core recursive expansion and size filtering logic.

## Complexity Analysis
- **Time Complexity**: $O(N \log N)$ for tree construction and $k$-NN searches. Performance is sensitive to the `cluster_tolerance` (larger $\tau$ increases search time).
- **Space Complexity**: $O(N)$ for the KD-Tree structure.

## Citations & Literature
- **Primary Method**: Rusu, R. B. (2010). *"Semantic 3D Object Maps for Everyday Manipulation."* PhD Thesis, Technische Universität München.
- **KD-Tree Foundation**: Bentley, J. L. (1975). *"Multidimensional binary search trees used for associative searching."* Communications of the ACM, 18(9), 509-517.
