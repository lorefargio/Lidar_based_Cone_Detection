# Ground Segmentation and Surface Estimation Analysis

## Theoretical Framework
Ground segmentation is the foundational stage of the perception pipeline, tasked with the binary classification of the point cloud $\mathcal{P}$ into ground $\mathcal{P}_g$ and non-ground $\mathcal{P}_o$ subsets. This implementation investigates the efficacy of geometric heuristics versus iterative plane-fitting models in high-dynamic environments.

### 1. Patchwork++: Adaptive and Robust Segmentation
Following the methodology proposed by Lee et al. (2022), Patchwork++ addresses the limitations of deterministic ground models (e.g., standard GPF or RANSAC) through a multi-stage adaptive process.

#### Concentric Zone Model (CZM)
To account for the non-uniform distribution of LiDAR points, the $\mathbb{R}^2$ plane is partitioned into concentric zones $\mathcal{Z}_m$ with varying radial and angular resolutions. This discretization mimics the sensor's angular resolution, assigning higher bin densities to the near-field where point density is maximum.

#### Reflected Noise Removal (RNR)
Virtual noise points originating from multi-path reflections (e.g., off vehicle hoods) often appear below the actual ground level, biasing plane estimation. RNR identifies these outliers by evaluating the intensity $\mathcal{I}$ and vertical coordinate $z$ of points in the bottom sensor rings:
$$\text{Noise if } z < h_{noise} \text{ and } \mathcal{I} < I_{noise}$$
where $h_{noise}$ is adaptively updated via A-GLE.

#### Region-wise Vertical Plane Fitting (R-VPF)
In urban or racing environments with vertical structures (e.g., retaining walls), standard Ground Plane Fitting (GPF) may treat wall-points as ground-seeds. R-VPF mitigates this by iteratively identifying vertical components through PCA and removing them from the bin-subset before final ground estimation.

#### Adaptive Ground Likelihood Estimation (A-GLE)
Rather than using static thresholds, A-GLE dynamically updates the elevation ($e_{\tau,m}$) and flatness ($f_{\tau,m}$) parameters for each zone $\mathcal{Z}_m$ based on the distribution of "definite ground" in previous frames. This enables the system to adapt to varying terrain types (e.g., transitioning from flat asphalt to bumpy grass).

### 2. Radial Slope Analysis (Slope-Based)
As an alternative to iterative fitting, this module evaluates the local geometric gradient $\nabla z$ along radial sectors.
*   **Mathematical Logic**: Given sorted points $\{p_1, \dots, p_n\}$ in a sector, $p_j$ is classified as ground if the slope $\alpha$ relative to the last confirmed ground point $p_{ref}$ satisfies:
    $$\alpha = \frac{|p_{j,z} - p_{ref,z}|}{||p_{j,xy} - p_{ref,xy}||} < \alpha_{max}$$
*   **Complexity**: $O(N \log N)$ due to the necessity of radial sorting per sector.
*   **Performance Note**: While less robust to noise than Patchwork++, the Slope-based approach provides lower latency and is highly effective for detecting the abrupt transition between the track and the cone base.

## Implementation Optimizations for Real-Time Control
1.  **Ordered Pre-processing**: Truncation and NaN removal are performed *prior* to any segmentation to ensure the VoxelGrid indices remain within integer bounds, preventing index overflow and associated I/O overhead.
2.  **Buffer Persistence**: All segmentation modules utilize pre-allocated, persistent memory structures to avoid the non-deterministic latency associated with runtime heap allocations.
