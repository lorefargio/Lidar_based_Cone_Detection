# Adaptive DBSCAN: Distance-Aware Density Clustering

The `AdaptiveDBSCANClusterer` is an advanced version of the DBSCAN algorithm, specifically engineered to compensate for the reduction in LiDAR point density as range increases. It employs a dynamic parameter model where both the radius $\epsilon$ and the density threshold $MinPts$ scale with radial distance.

## Abstract & Motivation
LiDAR sensors (e.g., Ouster OS1, Velodyne VLP-16) have a fixed angular resolution. Consequently, the distance between adjacent points in the same ring (azimuthal separation) and between rings (vertical separation) increases linearly with the range $r$. A fixed $\epsilon$ that works at 2m will fail at 20m. `AdaptiveDBSCAN` addresses this "beam divergence" effect.

## Mathematical Foundation
The clustering parameters $\epsilon$ and $MinPts$ are no longer constants but functions of the radial range $r = \sqrt{x^2 + y^2}$:
1. **Linear Epsilon Scaling**:
   $$\epsilon(r) = \epsilon_{base} + \alpha r$$
   where $\alpha$ is the growth coefficient.
2. **Exponential MinPts Decay**:
   $$\kappa(r) = \max\left(3, \kappa_{base} \cdot e^{-0.05r}\right)$$
   This ensures the density requirement is relaxed for distant objects while maintaining high noise rejection in the near-field.

## Implementation Walkthrough

### 1. Dynamic Parameter Lookup
During neighborhood search and cluster expansion, the `get_params(pt)` helper function is called:
```cpp
auto get_params = [&](const PointT& pt) {
    float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    float eps = eps_base_ + alpha_ * r;
    int min_p = std::max(3, static_cast<int>(min_pts_ * std::exp(-0.05f * r)));
    return std::make_pair(eps, min_p);
};
```

### 2. Conservative Grid Sizing
Since $\epsilon$ varies, the 3D flat grid's resolution must be fixed to the **maximum expected $\epsilon$** to ensure the 27-cell check remains exhaustive for all ranges.
```cpp
const float max_expected_eps = eps_base_ + alpha_ * 25.0f; // 25m cutoff
const float inv_grid_size = 1.0f / max_expected_eps;
```
If a cell's size is larger than the local $\epsilon$, the algorithm remains correct (it just checks slightly more points).

### 3. Adaptive Cluster Expansion
The core expansion logic is similar to `DBSCANClusterer`, but it re-evaluates `min_p` for every point being processed. This allows a cluster to bridge sparser regions if the density matches the range-based expectation.

## Complexity Analysis
- **Time Complexity**: **$O(N)$**. The dynamic lookup adds a small constant overhead (`std::sqrt`, `std::exp`), but the overall grid-based $O(1)$ lookup remains.
- **Space Complexity**: **$O(N + \text{GridVolume})$**. Similar to standard DBSCAN, but potentially slightly higher memory due to larger grid cells (sized to `max_expected_eps`).

## Citations & Literature
- **Epsilon Scaling**: Common technique in urban LiDAR segmentation. See: Lee, J. S., et al. (2022). *"Patchwork++: Fast and Robust Ground Segmentation."* for discussion on non-uniform point distributions.
- **Beam Divergence**: "LiDAR Intensity and Ranging Accuracy Analysis." in *Sensors* MDPI, discussing geometric spreading and its effect on point clustering.
