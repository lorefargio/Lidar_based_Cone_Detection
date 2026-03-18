# Formula Student LiDAR Perception (`fs_lidar_perception`)

Questo pacchetto ROS 2 implementa la pipeline di percezione LiDAR per veicoli autonomi di Formula Student. L'obiettivo principale è identificare in modo rapido e accurato i coni che delimitano il tracciato, elaborando nuvole di punti 3D ad alta frequenza (es. 10Hz - 20Hz).

Il progetto è strutturato per garantire **massima modularità** e fornire un **framework di valutazione delle performance**, fondamentale in assenza momentanea di Ground Truth per l'ottimizzazione dell'esecuzione in tempo reale.

---

## 🏗 Architettura e Modularità

Il codice sorgente è organizzato in sottomoduli logici, rendendo la manutenzione e l'espansione estremamente semplici. La vera forza del progetto risiede nell'uso di **Interfacce Astratte** C++ (`ClustererInterface`, `EstimatorInterface`) che permettono di sostituire l'algoritmo utilizzato a runtime semplicemente cambiando un parametro ROS, senza dover ricompilare il codice.

### Struttura delle Directory
- `include/` e `src/`:
  - `filtering/`: Rimozione del piano stradale (`GroundRemoverInterface`).
    - **Bin-based:** Veloce, basato su griglia polare e soglia di altezza locale.
    - **Slope-based:** Avanzato, analizza la pendenza radiale per preservare la base dei coni.
  - `clustering/`: Algoritmi per raggruppare i punti appartenenti a potenziali coni.
  - `estimation/`: Algoritmi per filtrare i cluster validi e stimare il centro del cono.
  - `utils/`: Strumenti di supporto, tra cui la profilazione delle performance (`PerformanceProfiler`).
  - `node/`: Il nodo ROS 2 principale (`perception_node`) che orchestra la pipeline.

### Algoritmi Implementati

#### 1. Clustering (`ClustererInterface`)
- **Grid Clusterer:** Veloce e basato su griglia voxel 2D.
- **Euclidean Clusterer:** Basato su KD-Tree classico (tramite PCL).
- **String Clusterer:** Ottimizzato per pattern di scansione ad anelli (Lidar rings).
- **DBSCAN & Adaptive DBSCAN:** Robusto al rumore e densità variabile.
- **Voxel Connected Components:** Approccio ibrido voxel/grafo ad alte prestazioni.

#### 2. Estimators (`EstimatorInterface`)
- **Rule-Based Estimator:** Classificazione euristica basata su dimensioni (bounding box) e densità di punti. Utilizza soglie dinamiche basate sulla distanza per ridurre i falsi positivi lontani.
- **Model Fitting Estimator:** Utilizza RANSAC per fittare la forma geometrica ideale di un cono e valutare la confidenza del cluster.

---

## ⏱ Benchmarking e Performance Profiling (Fase 4)

Attualmente (Fase 4), non avendo a disposizione rosbag annotati con *Ground Truth* per testare l'accuratezza pura (Precision/Recall), il focus principale dello sviluppo è sulle **prestazioni di esecuzione (Latenza e Jitter)**.

Il progetto include un **Framework di Benchmarking** integrato:
- **`PerformanceProfiler`**: Una classe C++ che traccia i tempi di esecuzione di ogni singola fase della pipeline (Ground Removal, Clustering, Estimation). Oltre alla media, calcola percentili critici per i sistemi real-time come il **P90, P95, e P99**.
- **Script di Automazione (`scripts/run_benchmarks.sh`)**: Permette di lanciare la pipeline iterando automaticamente su tutte le combinazioni di algoritmi.
- **Analisi e Plotting (`scripts/plot_metrics.py`)**: Estrae i report JSON generati dal nodo ROS 2 per produrre grafici (Boxplot, Bar chart) per confrontare la latenza e la stabilità degli algoritmi.

---

## 🚀 Come Testare Diversi Algoritmi

La pipeline può essere configurata interamente tramite file di launch. Puoi lanciare il nodo e testare combinazioni di algoritmi al volo:

```bash
# Esempio: Avvio con Clustering Euclideo e stima Rule-Based
ros2 launch fs_lidar_perception foxglove.launch.py clustering_algorithm:=euclidean estimator_type:=rule_based

# Esempio: Avvio con Adaptive DBSCAN e soglie dinamiche aggressive
ros2 launch fs_lidar_perception foxglove.launch.py clustering_algorithm:=adaptive_dbscan dynamic_width_decay:=0.05
```

### Parametri Supportati:
- `ground_remover_type`: `bin_based`, `slope_based`
- `clustering_algorithm`: `grid`, `euclidean`, `string`, `dbscan`, `adaptive_dbscan`, `voxel`
- `estimator_type`: `rule_based`, `model_fitting`
- `dynamic_width_decay`: (float) fattore di decadimento per le soglie dinamiche basate sulla distanza.

---

## 🛠 Prossimi Sviluppi
1. **Debugging & Finetuning dei parametri:** Affinare la precisione degli algoritmi di clustering implementati, trami te un finetuning dei parametri. 
2. **Raccolta Ground Truth:** Creazione di dataset per abilitare le metriche di accuratezza (True Positives, False Positives).
