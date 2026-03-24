# Formula Student LiDAR Perception 

Questo pacchetto implementa una pipeline di percezione LiDAR ad alte prestazioni per veicoli Formula Student, ottimizzata per l'identificazione rapida di coni in scenari dinamici.

---

## 🏗 Architettura della Pipeline

La pipeline è suddivisa in quattro stadi sequenziali e modulari:

1.  **[LiDAR Deskewing](docs/deskewing.md):** Correzione della distorsione temporale dei punti tramite interpolazione Slerp (400Hz IMU) e compensazione del braccio di leva (Lever Arm).
2.  **[Filtering (Ground Removal)](docs/filtering.md):** Separazione del suolo dagli ostacoli (Bin-based, Slope-based).
3.  **[Clustering (Object Grouping)](docs/clustering.md):** Raggruppamento dei punti in oggetti candidati (Grid, Euclidean, DBSCAN, Voxel).
4.  **[Estimation (Cone Classification)](docs/estimation.md):** Validazione geometrica (Rule-based, PCA, RANSAC) e calcolo della posizione XYZ.

---

## 🏎 Setup Hardware: Bi-Mensola (ZED2 + Hesai)

Il sistema è ottimizzato per un setup a supporto rigido ancorato al monoscocca in fibra di carbonio. 
*   **Sincronizzazione:** Pre-processing degli orientamenti IMU direttamente nel frame LiDAR per minimizzare il carico CPU (Manual Config).
*   **Compensazione Traslazionale:** Gestione dell'offset fisico tra la mensola della camera e quella del LiDAR per correggere le accelerazioni indotte dal beccheggio/rollio.

![Setup di testing e raccolta dati](/docs/testing_setup.jpg)


---

## 🚀 Guida Rapida all'Esecuzione

### Esecuzione Standard 
```bash
ros2 launch fs_lidar_perception foxglove.launch.py \
    use_deskewing:=true \
    imu_topic:=/zed/zed_node/imu/data \
    clustering_algorithm:=voxel
```

### Configurazione Parametri
Tutte le impostazioni del sistema (soglie, algoritmi, calibrazioni) sono documentate qui:
👉 **[Guida ai Parametri di Launch](docs/parameters.md)**

---

## ⏱ Benchmarking e Performance

Il progetto include un profiler integrato che salva i dati di latenza in `log_profiler/`.
*   **Target Real-Time:** 20Hz (50ms budget totale).
*   **Overhead Deskewing:** < 2ms (grazie all'ottimizzazione SIMD di Eigen e al pre-processing IMU).

---

## 🛠 Prossimi Sviluppi
*   **Calibrazione Fine:** Inserimento delle misure CAD precise per il braccio di leva IMU-LiDAR.
*   **Tuning:** Ottimizzazione delle soglie dei vari algoritmi.