# Parameter Specification and System Configuration

This document defines the configuration space for the LiDAR perception pipeline.

## 1. Global & Preprocessing
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `sensor_z` | `-0.52` | m | Mounting height of the LiDAR. |
| `max_range` | `25.0` | m | Max radial distance for processing. |
| **`voxel_size`** | **`0.035`** | **m** | **Early voxel grid leaf size.** |
| `min_cluster_size`| `2` | - | Min points per valid object. |
| `max_cluster_size`| `300` | - | Max points per valid object. |

## 2. Clustering (Grid-Based)
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| **`grid_resolution`** | **`0.12`** | **m** | **Spatial resolution for grouping.** |

## 3. Ground Segmentation (Slope-Based)
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `slope_max_slope` | `0.08` | dz/dr | Max gradient for ground. |
| `slope_segments` | `360` | - | Angular resolution. |

## 4. Estimation & Classification
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `rule_min_height` | `0.10` | m | Min cone height. |
| `rule_max_height` | `0.50` | m | Max cone height. |
| **`rule_min_points_cap`**| **`20`** | **-** | **Point count expectation cap.** |
| **`max_width_diff_ratio`**| **`3.5`** | **-** | **Symmetry (Symmetry/Shape) check.** |
| `pca_min_scatter` | `0.02` | - | Min scattering for 3D shape. |
| `verticality_min` | `0.45` | - | Min verticality threshold. |

## 5. Spatial Aggregation
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `tracking_match_dist`| `0.45` | m | Gating radius for spatial averaging. |
