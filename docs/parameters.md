# System Parameter Specification and Configuration Guide

This document lists and defines all ROS 2 parameters used to configure the LiDAR perception pipeline. These parameters map directly into the `PipelineConfig` struct during initialization.

---

## 1. Global & Preprocessing Parameters

| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `clustering_algorithm` | `"grid"` | - | Clustering method: `"grid"`, `"euclidean"`, `"dbscan"`, `"hdbscan"`, `"depth"`, `"voxel"`. |
| `ground_remover_type` | `"patchworkpp"` | - | Ground segmentation method: `"patchworkpp"`, `"slope_based"`, `"bin_based"`. |
| `sensor_z` | `-0.52` | m | Mounting height of the LiDAR sensor relative to the ground plane. |
| `max_range` | `25.0` | m | Maximum radial distance from the sensor for object detection. |
| `min_cluster_size` | `2` | points | Minimum number of points required to form a valid object cluster. |
| `max_cluster_size` | `300` | points | Maximum number of points allowed in a valid object cluster. |
| `use_voxel_filter` | `false` | - | If true, downsamples the deskewed obstacles using a PCL voxel grid filter. |
| `voxel_size` | `0.02` | m | Leaf size of the isotropic voxel grid filter. |
| `debug` | `true` | - | Toggles publication of high-overhead debugging markers and raw visualizer clouds. |
| `debug_pub_freq` | `10` | Hz | Target frequency for publishing visualization topics. |
| `log_dir` | `"log_profiler/"`| - | Output folder path for offline diagnostic JSON/CSV reports. |
| `profile_name` | `"default"` | - | Unique prefix naming key for saved log runs. |
| `log_clusters` | `true` | - | Saves cluster features dynamically into CSV logs for parameter tuning. |
| `log_all_clusters` | `false` | - | Logs all processed clusters, including rejected noise candidates. |

---

## 2. Ground Segmentation Specific Parameters

### 2.1 Patchwork++
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `pw_num_iter` | `3` | - | Number of concentric zone iterations. |
| `pw_th_dist` | `0.02` | m | Distance threshold for flat ground points classification. |
| `pw_th_seeds` | `0.02` | m | Threshold for initial ground seeds collection. |
| `pw_th_dist_v` | `0.1` | m | Vertical zone height variance threshold. |
| `pw_th_seeds_v` | `0.02` | m | Zone seeds threshold for vertical variance sectors. |
| `pw_min_range` | `0.5` | m | Minimum inner radius boundary. |
| `pw_uprightness_thr` | `0.707` | - | Normal vector cosine threshold for terrain tilt. |
| `pw_enable_RNR` | `true` | - | Enables Reflected Noise Removal (drops low-intensity returns). |
| `pw_enable_TGR` | `true` | - | Enables Temporal Ground Reversion. |

### 2.2 Slope-Based
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `slope_max_slope` | `0.08` | dz/dr | Maximum radial gradient incline slope. |
| `slope_max_z_diff` | `0.05` | m | Maximum vertical drop variance. |
| `slope_initial_threshold`| `0.05` | m | Core base distance threshold. |
| `slope_segments` | `360` | segments | Angular division resolution. |

### 2.3 Bin-Based
| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `bin_local_threshold` | `0.02` | m | Relative height difference limit in bins. |
| `bin_hard_cutoff` | `-0.47` | m | Absolute base height ceiling cutoff. |
| `bin_segments` | `500` | segments | Radial polar grid segments. |
| `bin_bins` | `500` | bins | Concentric divisions per segment. |

---

## 3. Clustering Specific Parameters

| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `grid_resolution` | `0.12` | m | Spatial resolution (cell dimensions) for grid clustering. |
| `euclidean_tolerance` | `0.35` | m | Search radius tolerance for Euclidean KD-Tree extraction. |
| `dbscan_eps` | `0.30` | m | Search radius (Epsilon) parameter for DBSCAN. |
| `dbscan_min_pts` | `3` | points | Core point minimum density requirement for DBSCAN. |
| `hdbscan_min_pts` | `5` | points | Minimum neighborhood point density constraint for HDBSCAN. |
| `hdbscan_alpha` | `0.02` | - | Cluster persistence scale-factor parameter. |
| `voxel_grid_size` | `0.02` | m | Isotropic leaf sizing for voxel components grouping. |
| `depth_theta_thr` | `10.0` | deg | Angle threshold for range-image topological BFS grouping. |
| `depth_num_rings` | `32` | rings | Vertical laser channel division multiplier matching LiDAR beams. |

---

## 4. Estimation & PCA Classification Parameters

| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `pca_max_linearity` | `0.88` | - | Maximum allowed PCA linearity feature value (rejects flat bars). |
| `pca_max_planarity` | `0.80` | - | Maximum allowed PCA planarity feature value (rejects panels). |
| `pca_min_scatter` | `0.02` | - | Minimum allowed 3D scattering value (cone bulk requirement). |
| `pca_min_verticality` | `0.65` | - | Minimum verticality threshold (alignment along Z-axis). |
| `rule_min_height` | `0.10` | m | Minimum allowed physical object height. |
| `rule_max_height` | `0.80` | m | Maximum allowed physical object height (Formula Student large limit). |
| `rule_base_min_width` | `0.10` | m | Minimum bounding box width at base. |
| `rule_max_width` | `0.50` | m | Maximum allowed bounding box width. |
| `rule_dynamic_width_decay`| `0.005`| m/m | Rate of width constraint decay per meter of range. |
| `rule_min_points_at_10m` | `5` | points | Point density floor constraint normalized to 10m range. |
| `rule_min_points_cap` | `60` | points | Maximum point constraint cap at close distance. |
| `rule_min_intensity` | `5.0` | - | Minimum average intensity of returned points. |

---

## 5. Merging & Deskewing Parameters

| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `merge_dist` | `0.25` | m | Volumetric merging distance for unifying fragmented components. |
| `tracking_match_dist` | `0.45` | m | Spatial aggregation search radius (unweighted spatial mean). |
| `use_deskewing` | `true` | - | Enables point-wise IMU ego-motion compensation. |
| `imu_topic` | `"/zed/zed_node/imu/data"` | - | IMU sensor subscription topic name. |
| `imu_frame` | `"zed_imu_link"` | - | TF frame ID of the IMU sensor. |
| `imu_lowpass_cutoff` | `15.0` | Hz | Low-pass filter frequency boundary for raw IMU rates. |
| `extrinsic_rotation` | *(Matrix)* | rad | 9-element 3x3 rotation mapping LiDAR to IMU frame. |
| `extrinsic_translation`| *(Array)* | m | 3-element translation offset vector from IMU to LiDAR. |
| `roll_deg` | `-0.4` | deg | Dynamic correction roll angle offset. |
| `pitch_deg` | `0.0` | deg | Dynamic correction pitch angle offset. |
| `yaw_deg` | `0.0` | deg | Dynamic correction yaw angle offset. |
| `world_up_axis` | `"y"` | - | Gravity axis orientation reference (`"y"` or `"z"`). |
