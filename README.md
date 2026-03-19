# Lidar Based cone detection

Questo pacchetto ROS 2 implementa la pipeline di percezione LiDAR per veicoli autonomi di Formula Student. L'obiettivo principale è identificare in modo rapido e accurato i coni che delimitano il tracciato, elaborando nuvole di punti 3D ad alta frequenza (es. 10Hz - 20Hz).

Il progetto è strutturato per garantire **massima modularità** e fornire un **framework di valutazione delle performance**, fondamentale per l'ottimizzazione dell'esecuzione in tempo reale su hardware embedded.

---

## 🏗 Architettura della Pipeline

La pipeline è suddivisa in tre stadi principali intercambiabili a runtime tramite parametri ROS:

1.  [**Filtering (Ground Removal)**](docs/filtering.md): Separazione del suolo dagli ostacoli (Bin-based, Slope-based, Patchwork++).
2.  [**Clustering (Object Grouping)**](docs/clustering.md): Raggruppamento dei punti ostacolo in oggetti (Grid, Euclidean, String, DBSCAN, Voxel).
3.  [**Estimation (Cone Classification)**](docs/estimation.md): Validazione dei cluster e stima della posizione XYZ dei coni (Rule-based, PCA, RANSAC).
4.  [**Fusion Integration**](docs/fusion_integration.md): Supporto avanzato per la fusione Camera-LiDAR (Range/Bearing e Full Point Cloud).

---

## ⏱ Benchmarking e Performance Profiling

Il progetto include un sistema integrato per misurare e confrontare le performance di ogni fase in termini di latenza, jitter e stabilità.

*   [**Framework di Testing e Benchmarking**](docs/benchmarking.md): Dettagli sull'analisi statistica (P95, P99) e sugli script di automazione.

---

## 🚀 Come Eseguire il Sistema

La pipeline può essere configurata interamente tramite file di launch. Puoi lanciare il nodo e testare combinazioni di algoritmi al volo, prima è necessario aver installato foxglove e avere un abiente che supporta ROS2, nello specifico il codice è stato testato su unn dispositivo con Ubunutu 22.04 e ROS2 Humble:

```bash
# Esempio: Avvio con Patchwork++ per il suolo e Clustering Voxel
ros2 launch fs_lidar_perception foxglove.launch.py \
    ground_remover_type:=patchwork \
    clustering_algorithm:=voxel \
    estimator_type:=rule_based
```

### Parametri Supportati:
- `ground_remover_type`: `bin_based`, `slope_based`, `patchwork`, `patchwork_official`
- `clustering_algorithm`: `grid`, `euclidean`, `string`, `dbscan`, `hdbscan`, `voxel`
- `estimator_type`: `rule_based`, `ransac`
---

## 🛠 Prossimi Sviluppi
1.  **Debugging & Finetuning:** Affinamento continuo dei parametri per i vari algoritmi, debugging dell'estimazione tramite ransac.
