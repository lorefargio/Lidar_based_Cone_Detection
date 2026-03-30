# Multimodal Sensor Fusion Integration (Camera-LiDAR)

## Abstract
This document delineates the architectural interfaces designed to facilitate synchronous fusion between LiDAR-derived 3D spatial data and Camera-derived 2D semantic labels. The objective is to enhance cone classification reliability by cross-validating geometric primitives with chromatic information.

## Data Interface Specification

### 1. Perceptual Primitives
The `Cone` structure is designed as a shared primitive, encapsulating both Cartesian coordinates ($x, y, z$) and polar descriptors (Range $\rho$, Bearing $\theta$). This dual representation simplifies the projection into the image plane ($u, v$):
$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \mathbf{K} \cdot \mathbf{T}_{CL} \cdot \begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix}$$
where $\mathbf{K}$ is the intrinsic camera matrix and $\mathbf{T}_{CL}$ is the extrinsic LiDAR-to-Camera transformation.

### 2. Chromatic Attribution
The `ConeColor` enumeration provides a semantic link to Camera-based object detection. During fusion, the system assigns a color label ($\text{Blue, Yellow, Orange}$) based on the highest intersection-over-union (IoU) between the projected LiDAR cluster bounding box and the 2D neural network detection.

## Output Streams for Fusion

### /perception/cones (Centroid Data)
Publishes the localized centroids. The `intensity` field is repurposed to carry the radial range $\rho$, providing a direct high-bandwidth channel for SLAM algorithms without requiring additional message deserialization.

### /perception/cone_points (Cluster Volumes)
Publishes the union of all points $\mathcal{P} \in \mathcal{C}_k$ for validated cones. This stream is critical for **Contour-based Fusion**, allowing for precise alignment checks against the semantic segments in the image.

## Visualization Interface
Visual validation is performed via **Textual Metadata Markers** in the `distance_labels` namespace. These markers display real-time radial distance with centimeter precision, enabling rapid experimental debugging of the depth-estimation accuracy.
