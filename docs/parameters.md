# Guida ai Parametri di Launch e Configurazione

Questa guida elenca tutti i parametri dichiarati nel `foxglove.launch.py` e nel nodo `fs_lidar_perception`. I parametri consentono il tuning fine della pipeline senza ricompilazione.

---

## 1. Parametri Generali e Comuni

| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `bag` | `''` | Percorso alla rosbag da riprodurre. |
| `clustering_algorithm` | `grid` | Algoritmo: `grid`, `euclidean`, `string`, `dbscan`, `hdbscan`, `voxel`. |
| `ground_remover_type` | `slope_based` | Algoritmo: `bin_based`, `slope_based`, `patchworkpp`. |
| `estimator_type` | `rule_based` | Algoritmo: `rule_based`, `ransac`. |
| `sensor_z` | `-0.52` | Altezza del LiDAR dal suolo (m). Cruciale per il calcolo dell'altezza dei coni. |
| `max_range` | `25.0` | Distanza massima di elaborazione (m). |
| `min_cluster_size` | `3` | Numero minimo di punti per formare un cluster. |
| `max_cluster_size` | `300` | Numero massimo di punti (per filtrare muri o auto). |

---

## 2. LiDAR Deskewing & Filters

| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `use_deskewing` | `true` | Abilita la correzione della distorsione da movimento. |
| `imu_topic` | `/zed/zed_node/imu/data` | Topic IMU per i dati di orientamento. |
| `deskew_use_translation` | `true` | Abilita la compensazione del braccio di leva (Lever Arm). |
| `static_imu_to_lidar_xyz` | `[0.0, 0.0, 0.0]` | Offset XYZ tra IMU e LiDAR (m). |
| `use_voxel_filter` | `false` | Abilita il downsampling post-ground removal. |
| `voxel_size` | `0.05` | Dimensione della cella per il filtro voxel (m). |

---

## 3. Ground Removal (Filtering)

### Bin-Based
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `bin_local_threshold` | `0.02` | Altezza massima sopra il minimo del bin per essere suolo (m). |
| `bin_hard_cutoff` | `-0.47` | Quota z assoluta sotto la quale tutto è suolo. |
| `bin_segments` | `500` | Numero di settori angolari nella griglia polare. |
| `bin_bins` | `500` | Numero di bin radiali nella griglia polare. |

### Slope-Based
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `slope_max_slope` | `0.08` | Pendenza massima (dz/dr) tra punti consecutivi di suolo. |
| `slope_max_z_diff` | `0.05` | Salto in Z massimo tra punti consecutivi di suolo. |
| `slope_initial_threshold`| `0.05` | Tolleranza iniziale vicino al sensore (m). |
| `slope_segments` | `360` | Numero di settori angolari per l'analisi. |

### Patchwork++
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `pw_num_iter` | `3` | Iterazioni per la stima del piano del terreno. |
| `pw_th_dist` | `0.02` | Soglia di distanza dal piano stimato (m). |
| `pw_min_range` | `0.5` | Distanza minima di elaborazione (m). |
| `pw_uprightness_thr` | `0.707` | Soglia di verticalità per la normale del piano. |
| `pw_enable_RNR` | `true` | Abilita Reflected Noise Removal. |

---

## 4. Clustering (Object Grouping)

### Algoritmi Standard
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `euclidean_tolerance` | `0.35` | Raggio di ricerca per la connettività (m). |
| `dbscan_eps` | `0.30` | Raggio epsilon per la densità (DBSCAN). |
| `dbscan_min_pts` | `3` | Punti minimi per essere un core-point. |
| `voxel_grid_size` | `0.15` | Dimensione della cella 3D per Connected Components (m). |

### HDBSCAN (Adaptive DBSCAN)
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `hdbscan_eps_base` | `0.15` | Epsilon base a distanza 0m. |
| `hdbscan_alpha` | `0.015` | Coefficiente di crescita di epsilon con la distanza. |

### String Clusterer
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `string_max_dist` | `0.3` | Distanza massima tra punti consecutivi nello sweep. |
| `string_max_int_jump` | `100.0` | Salto di intensità massimo per mantenere il cluster. |

---

## 5. Estimation (Cone Classification)

### PCA & Rule-Based
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `pca_max_linearity` | `0.8` | Scarta oggetti con linearità superiore (es. paletti). |
| `pca_max_planarity` | `0.8` | Scarta oggetti piatti (es. muri). |
| `pca_min_scatter` | `0.02` | Assicura che l'oggetto sia volumetrico. |
| `rule_min_height` | `0.10` | Altezza minima del cono (m). |
| `rule_max_height` | `0.50` | Altezza massima del cono (m). |
| `rule_base_min_width` | `0.10` | Larghezza minima alla base (m). |
| `rule_dynamic_width_decay`| `0.005` | Riduzione della larghezza attesa per metro di distanza. |
| `rule_min_points_at_10m` | `10` | Numero di punti minimi a 10 metri di distanza. |
| `rule_min_intensity` | `5.0` | Intensità riflessa media minima. |

### RANSAC (Model Fitting)
| Parametro | Default | Descrizione |
| :--- | :--- | :--- |
| `ransac_dist_threshold` | `0.05` | Distanza massima punto-superficie cilindrica. |
| `ransac_max_iter` | `1000` | Numero massimo di tentativi RANSAC. |
| `ransac_radius_min` | `0.05` | Raggio minimo del modello cilindrico (m). |
| `ransac_radius_max` | `0.18` | Raggio massimo del modello cilindrico (m). |
| `ransac_min_inlier_ratio` | `0.4` | Percentuale minima di punti sul modello. |
