# Geometric Estimation and Cone Classification

## Theoretical Overview
The estimation stage evaluates candidate clusters $\mathcal{C}_k$ using geometric primitives and statistical features to determine the probability of them being traffic cones.

### Covariance-Based Shape Analysis
We perform Eigen-decomposition on the 3D covariance matrix $\Sigma$ of each cluster:
$$\Sigma = \frac{1}{n} \sum_{i=1}^{n} (p_i - \bar{p})(p_i - \bar{p})^T$$
From the resulting eigenvalues $\lambda_1 \geq \lambda_2 \geq \lambda_3$, we derive local descriptors:
1.  **Linearity ($L$):** $(\lambda_1 - \lambda_2) / \lambda_1$ (Detects posts/poles).
2.  **Planarity ($P$):** $(\lambda_2 - \lambda_3) / \lambda_1$ (Detects walls/surfaces).
3.  **Scattering ($S$):** $\lambda_3 / \lambda_1$ (Ensures volumetric consistency).

## Implementation Optimizations

### 1. Direct Eigen-Solver vs. General PCA
Standard PCL PCA classes are designed for general-purpose applications and include projection steps that are redundant for our needs. Our implementation uses `Eigen::SelfAdjointEigenSolver` directly on the $3 \times 3$ covariance matrix.
*   **Justification**: For small clusters (typical of cones, 3-30 points), the direct solver is significantly more efficient and avoids unnecessary memory allocations.

### 2. Bounding Box Early-Exit
Before performing the computationally expensive Eigen-decomposition, a preliminary geometric check is performed:
*   **Rule**: Scart candidates where $Height \notin [H_{min}, H_{max}]$ or $Width > W_{max}$.
*   **Justification**: Reduces the number of clusters undergoing PCA by filter out large environmental structures (walls, cars) or ground artifacts in $O(N)$ time.

### 3. Voxel-Aware Thresholding
Due to the mandatory 5cm voxelization in the preprocessing stage, the dynamic point count threshold is capped:
$$\text{TargetPoints} = \min(30, \max(3, \text{PointsAt10m} \cdot \frac{100}{r^2}))$$
*   **Justification**: A cone has a finite surface area. With 5cm voxels, it cannot physically contain more than ~30 points. Capping this expectation prevents the misclassification of close-range cones.

## Classification Logic
The system currently employs a **Rule-Based Bayesian-like** approach where geometric features (PCA, Dimensions, Intensity) are weighed to generate a confidence score $\Psi \in [0, 1]$.
