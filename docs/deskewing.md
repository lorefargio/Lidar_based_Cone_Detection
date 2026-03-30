# Motion Compensation and LiDAR Deskewing

## Theoretical Overview
LiDAR sensors acquire points sequentially over a sweep duration $T_{sweep}$. For a vehicle moving at high velocity $v$, points acquired at $t=0$ and $t=T_{sweep}$ are spatially inconsistent. This distortion (skewing) can lead to significant localization errors and PCA failure in classification.

### Mathematical Formulation
For each point $p_i$ acquired at time $t_i \in [t_{start}, t_{end}]$, we compute the relative rotation $\Delta \mathbf{R}_i$ from the IMU data:
$$\Delta \mathbf{R}_i = \mathbf{Q}_{start}^{-1} \cdot \mathbf{Q}_{interp}(t_i)$$
The corrected point $p_i'$ is calculated as:
$$p_i' = \Delta \mathbf{R}_i \cdot (p_i + \mathbf{L}) - \mathbf{L}$$
where $\mathbf{L}$ is the lever-arm vector between the IMU and LiDAR centers.

## Implementation Specifics

### 1. Fast NLERP Interpolation
While Spherical Linear Interpolation (Slerp) is mathematically rigorous, our implementation utilizes **Normalized Linear Interpolation (Nlerp)** for point-wise correction.
*   **Justification**: For the small angular intervals between high-frequency IMU samples (400Hz), Nlerp is visually indistinguishable from Slerp but avoids the expensive trigonometric functions required by the latter.

### 2. SIMD-Parallelized Transformation Loop
The transformation loop is the primary candidate for SIMD optimization. We use **OpenMP** (`#pragma omp parallel for`) to distribute point corrections across CPU cores.
*   **Optimization**: We leverage Eigen's alignment and SSE/AVX instructions to ensure that the coordinate rotation $\mathbf{R} \cdot p$ is performed in parallel.

### 3. Monotonic Timestamp Search
To find the bounding IMU samples for a given $t_i$, we exploit the chronological order of LiDAR points. Instead of binary search $O(N \log M)$, we use a **Moving Iterator** search $O(N + M)$.
*   **Justification**: Reduces the average search time to $O(1)$ per point, ensuring that deskewing overhead remains below 2ms.

## Rigorous Synchronization
The system requires PTP/NTP synchronization between the LiDAR and the host PC. A drift of $>5ms$ in timestamps would invalidate the motion compensation, leading to increased perception jitter.
