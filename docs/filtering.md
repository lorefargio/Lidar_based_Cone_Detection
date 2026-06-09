# Ground Segmentation and Surface Estimation Analysis

Ground segmentation divides the pre-processed and deskewed point cloud $\mathcal{P}$ into two disjoint subsets: the traversable ground surface $\mathcal{P}_g$ and the non-ground obstacles $\mathcal{P}_o$. This implementation supports three distinct algorithms.

---

## 1. Patchwork++: Adaptive and Robust Segmentation
Patchwork++ represents the state-of-the-art (SOTA) in terrain segmentation, addressing the limitations of traditional GPF (Ground Plane Fitting) or RANSAC through regional estimation and outlier rejection.

### Concentric Zone Model (CZM)
To handle the variable density of LiDAR returns, the horizontal plane is divided into Concentric Zones ($\mathcal{Z}$) with higher angular and radial resolutions close to the sensor (where density is high) and coarser resolutions far away:
*   **Zone 0 (Near-field)**: Dense binning to capture fine terrain variations.
*   **Zone 3 (Far-field)**: Coarse binning to estimate sparse returns.

### Reflected Noise Removal (RNR)
Multi-path reflections (e.g., off the car's body) appear as virtual points beneath the ground, biasing plane fitting. RNR filters these by checking the intensity $\mathcal{I}$ and height $z$ of returns:
$$\text{Noise if } z < h_{\text{noise}} \text{ and } \mathcal{I} < I_{\text{noise}}$$

### Region-wise Vertical Plane Fitting (R-VPF)
In the presence of vertical structures (e.g. concrete walls or tyre barriers), points on the walls can corrupt the initial ground plane seeds. R-VPF identifies vertical components via PCA and filters them out before performing the final ground estimation.

### Adaptive Ground Likelihood Estimation (A-GLE)
Instead of static height limits, A-GLE dynamically updates the height margin ($e_{\tau,m}$) and flat-surface threshold ($f_{\tau,m}$) for each concentric zone based on the distribution of definite ground in previous scans, adapting to track slopes and elevation changes.

---

## 2. Radial Slope Analysis (Slope-Based)
The `SlopeBasedGroundRemover` segments points by analyzing the local geometric gradient along angular slices.

*   **Mathematical Logic**:
    The point cloud is partitioned into $S$ sectors (e.g. 360 slices). Within each slice, points are sorted by radial distance. A point $p_j$ is classified as ground if the slope relative to the last confirmed ground point $p_{\text{ref}}$ is below a maximum incline threshold $\alpha_{\text{max}}$:
    $$\text{Slope } \alpha = \frac{|p_{j,z} - p_{\text{ref},z}|}{\|p_{j,xy} - p_{\text{ref},xy}\|} < \alpha_{\text{max}}$$
*   **Evaluation**:
    *   *Complexity*: $\mathcal{O}(N \log N)$ due to sorting per sector.
    *   *Pros*: Highly effective for finding the sharp transition at the base of the cones.
    *   *Cons*: Sensitive to noise and sensor range fluctuations.

---

## 3. Polar Bin-Based Grid (Bin-Based)
The `BinBasedGroundRemover` is a fast grid-discretized segmentation module.

*   **Mathematical Logic**:
    The horizontal space is partitioned into a polar grid of segments (radial sectors) and bins (concentric rings). Each point is mapped to coordinates $(s, b)$:
    $$s = \lfloor \theta / \Delta\theta \rfloor, \quad b = \lfloor r / \Delta r \rfloor$$
    Within each polar cell $(s, b)$, the algorithm computes the minimum vertical coordinate:
    $$z_{\text{min}}(s, b) = \min_{p \in \text{cell}(s, b)} p_z$$
    A point $p$ in cell $(s, b)$ is segmented as ground if it is close to this minimum and lies below a hard cutoff:
    $$p \in \mathcal{P}_g \quad \text{if} \quad p_z - z_{\text{min}}(s, b) \leq \text{threshold}_{\text{local}} \quad \text{and} \quad p_z \leq \text{cutoff}_{\text{hard}}$$
*   **Evaluation**:
    *   *Complexity*: Strictly $\mathcal{O}(N)$ for cell binning and checking.
    *   *Pros*: Blazing fast; very effective on flat surfaces.
    *   *Cons*: Struggles on vertical transitions or slopes.

---

## 4. Implementation Details
1.  **Buffer Persistence**: Pre-allocated memory blocks are used across all segmentation algorithms to prevent dynamic heap allocation latencies.
2.  **Voxel Grid Downsampling**: An optional voxel filter can downsample the isolated obstacle cloud $\mathcal{P}_o$ afterwards to keep the clustering workload low.
