# HDBSCAN: Hierarchical Lidar-Aware Clustering

The `HDBSCANClusterer` implemented here is a specialized version of the Hierarchical Density-Based Spatial Clustering algorithm, specifically engineered to overcome the "Near-Field Bias" of traditional density-based methods.

## Abstract & Motivation
In autonomous racing, traffic cones are distributed from 2m to 25m. Due to the angular resolution of the LiDAR (e.g., Hesai Pandar40P), the point density at 25m is orders of magnitude lower than at 2m. 
Standard HDBSCAN often fails to detect distant cones because their low absolute stability causes them to be labeled as noise in the global hierarchy. Our **Range-Normalized HDBSCAN** achieves distance invariance by scaling the distance metric according to the sensor's beam divergence.

## Mathematical Foundation

### 1. Distance Normalization (Hesai-Model)
To maintain uniform density across the field, we transform every Euclidean distance $d(u,v)$ using a range-dependent scaling factor:
$$d_{norm}(u, v) = \frac{d(u, v)}{1 + \alpha \cdot \text{range}(u)}$$
where $\alpha$ (alpha_scaling) is tuned to the angular resolution of the sensor. This ensures that a 40cm gap at 20m "looks" as dense as a 10cm gap at 5m.

### 2. Normalized Mutual Reachability Distance (MRD)
The core of HDBSCAN is the MRD, which we normalize to ensure scale-invariant core distances:
$$d_{mreach}(u, v) = \max(\text{core}_k(u)_{norm}, \text{core}_k(v)_{norm}, d_{norm}(u, v))$$
This allows the algorithm to build a hierarchy where "stability" is a measure of relative geometric integrity rather than absolute point count.

## Implementation Walkthrough

### 1. Core Distance Calculation
We use a **KD-Tree** to find the $k$-th neighbor. The result is immediately normalized by range:
```cpp
float raw_core_dist = std::sqrt(nn_dists.back());
core_distances[i] = raw_core_dist / (1.0f + config_.alpha_scaling * r);
```

### 2. K-NN Graph & Forest Construction
Instead of a single global tree, we build a **Minimum Spanning Forest** using a 10-NN graph. This ensures that every isolated cono has its own branch in the hierarchy, preventing small clusters from being swallowed by the global noise floor.

### 3. Early Stability Extraction
During the Single Linkage process (Kruskal-like), we monitor the size of merging components. If a component reaches the size of a cone ($3-300$ points) and its next merge would exceed this limit, we **extract it immediately**. This preserves the integrity of distant, sparse clusters.

## Complexity Analysis
- **Time Complexity**: $O(N \log N)$ for KD-Tree search + $O(N \cdot M \log(N \cdot M))$ for sorting the K-NN graph edges.
- **Space Complexity**: $O(N \cdot M)$ to store the proximity graph.

## Citations & Literature
- **Foundation**: Campello, R. J., Moulavi, D., & Sander, J. (2013). *"Density-based clustering based on hierarchical density estimates."*
- **Lidar Adaptation**: Inspired by "Distance-dependent Thresholding" in urban navigation tasks.
