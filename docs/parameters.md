# Guida ai Parametri di Launch e Configurazione

Questa guida elenca tutti i parametri dichiarati nel `foxglove.launch.py` e le loro implicazioni operative nel nodo di percezione.

---

## 1. Parametri Generali e di Input

| Parametro | Valore Default | Descrizione |
| :--- | :--- | :--- |
| `bag` | `''` | Percorso opzionale per eseguire una rosbag in loop all'avvio del nodo. |
| `use_voxel_filter` | `false` | Se attivo, applica un filtro di downsampling alla nuvola ostacoli dopo la rimozione del suolo. |
| `voxel_size` | `0.05` | Dimensione del voxel (m) per il filtro di downsampling. |

---

## 2. LiDAR Deskewing (ZED IMU)

| Parametro | Valore Default | Descrizione |
| :--- | :--- | :--- |
| `use_deskewing` | `true` | Abilita/Disabilita la correzione della distorsione da movimento. |
| `imu_topic` | `/zed/zed_node/imu/data` | Topic ROS 2 dell'IMU della camera ZED. |
| `static_imu_to_lidar` | `[0, 0, 0]` | Offset traslazionale [X, Y, Z] tra l'IMU e il LiDAR (meters). |

---

## 3. Ground Removal (Filtering)

| Parametro | Valore Default | Descrizione |
| :--- | :--- | :--- |
| `ground_remover_type` | `slope_based` | Algoritmo per il suolo: `bin_based` o `slope_based`. |
| `sensor_z` | `-0.52` | Altezza del LiDAR dal terreno (fondamentale per Slope-based). |
| `max_slope` | `0.08` | (Slope-based) Pendenza massima accettabile tra punti consecutivi di suolo. |

---

## 4. Clustering (Object Grouping)

| Parametro | Valore Default | Descrizione |
| :--- | :--- | :--- |
| `clustering_algorithm` | `grid` | Algoritmo: `grid`, `euclidean`, `string`, `dbscan`, `hdbscan`, `voxel`. |
| `min_cluster_size` | `3` | Punti minimi per considerare un cluster come oggetto. |
| `max_cluster_size` | `300` | Punti massimi (per scartare muri o grandi ostacoli). |

---

## 5. Estimation (Cone Classification)

| Parametro | Valore Default | Descrizione |
| :--- | :--- | :--- |
| `estimator_type` | `rule_based` | Algoritmo: `rule_based` o `ransac`. |
| `pca_max_linearity` | `0.8` | Soglia PCA: valori superiori scartano oggetti lineari (paletti/gambe). |
| `pca_min_scatter` | `0.005` | Soglia PCA: assicura che l'oggetto sia volumetrico (un vero cono). |
| `dynamic_width_decay` | `0.005` | Riduzione della larghezza attesa del cono con la distanza radiale. |

---
