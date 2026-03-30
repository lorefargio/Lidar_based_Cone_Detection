# Parameter Specification and System Configuration

## Overview
This document defines the configuration space for the LiDAR perception pipeline. Parameters are exposed via the ROS 2 parameter system and can be tuned at runtime or via YAML configuration files.

## 1. Preprocessing & Global Constants
| Parameter | Type | Default | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `sensor_z` | `double` | `-0.52` | m | Mounting height of the LiDAR optical center relative to the ground. |
| `max_range` | `double` | `25.0` | m | Maximum radial distance for processing. Points beyond this are truncated. |
| `min_cluster_size` | `int` | `3` | - | Cardinality threshold for candidate objects. |
| `max_cluster_size` | `int` | `300` | - | Upper bound filter to scart large environmental structures. |

## 2. Temporal Sync & Deskewing
| Parameter | Type | Default | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `use_deskewing` | `bool` | `true` | - | Enables motion compensation using high-frequency IMU data. |
| `imu_topic` | `string` | `/zed/imu` | - | Source topic for orientation quaternions. |
| `static_imu_to_lidar_xyz` | `vec3` | `[0,0,0]`| m | Translational offset (lever-arm) between IMU and LiDAR. |

## 3. Ground Segmentation (Slope-Based)
| Parameter | Type | Default | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `slope_max_slope` | `double` | `0.08` | dz/dr | Maximum geometric slope tolerated for ground points. |
| `slope_max_z_diff` | `double` | `0.05` | m | Maximum altitude jump between consecutive radial points. |
| `slope_segments` | `int` | `360` | - | Angular resolution for radial sector sorting. |

## 4. Optimized Clustering (DBSCAN)
| Parameter | Type | Default | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `dbscan_eps` | `double` | `0.30` | m | Search radius for density-based grouping. |
| `dbscan_min_pts` | `int` | `3` | - | Minimum points to identify a core cluster (Voxel-aware). |
| `hdbscan_alpha` | `double` | `0.015`| - | Epsilon growth factor for adaptive distance scaling. |

## 5. Classification & Shape Analysis
| Parameter | Type | Default | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `rule_min_height` | `double` | `0.10` | m | Minimum vertical extent of a traffic cone. |
| `rule_max_height` | `double` | `0.50` | m | Maximum vertical extent of a traffic cone. |
| `pca_max_linearity` | `double` | `0.8` | - | Scart threshold for post-like objects. |
| `pca_min_scatter` | `double` | `0.02` | - | Threshold to ensure volumetric tridimensionality. |
| `rule_min_points_at_10m`| `int` | `5` | - | Voxel-aware expectation for cone point count. |

## 6. Communication & Visualization
| Parameter | Type | Default | Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| `debug_pub_freq` | `int` | `10` | frames| Decimation factor for publishing high-bandwidth PointClouds. |
