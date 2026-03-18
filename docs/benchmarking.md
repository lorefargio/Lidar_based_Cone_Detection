# Framework di Testing e Benchmarking

In un sistema real-time a 20Hz (50ms per frame), non è sufficiente "funzionare". Bisogna funzionare con latenza prevedibile e jitter minimo. Il nostro framework è progettato per misurare oggettivamente le performance di ogni algoritmo e fase della pipeline.

## 1. Architettura del Benchmarking

Il framework si compone di tre parti principali che lavorano in modo coordinato:

### 1.1 PerformanceProfiler (C++)
È una classe integrata direttamente nel nodo ROS 2 (`LidarPerceptionNode`).
*   **Timer Ad Alta Risoluzione:** Utilizza `std::chrono::high_resolution_clock` per misurare il tempo di esecuzione di ogni singola fase (`ground_removal`, `clustering`, `estimation`).
*   **Gestione Frame:** Accumula i dati di ogni frame in memoria durante l'intera durata del test per evitare l'overhead di scrittura su file durante l'elaborazione.
*   **Export JSON:** Al termine dell'esecuzione (nel distruttore), salva i dati grezzi in un file JSON strutturato (es. `profiler_grid.json`).

### 1.2 Script di Automazione (`run_benchmarks.sh`)
Automatizza l'esecuzione dei test su diverse configurazioni:
1.  Avvia una rosbag in loop.
2.  Lancia il nodo con un algoritmo specifico (es. `clustering_algorithm:=dbscan`).
3.  Attende la fine della riproduzione, chiude il nodo in modo pulito (SIGINT) per far scattare il salvataggio dei log.
4.  Ripete per ogni combinazione desiderata.

### 1.3 Analisi Dati e Grafici (`plot_metrics.py`)
Uno script Python (`pandas` + `seaborn`) che legge i file JSON generati e produce grafici professionali (Publication-Ready).

---

## 2. Metriche Fondamentali

Per valutare un sistema real-time, non usiamo solo la media (Mean), ma i percentili:

*   **P50 (Median):** Il tempo tipico di elaborazione.
*   **P90 / P95 / P99:** Indicano i casi peggiori. Un `P99 = 45ms` significa che il 99% dei frame rispetta il limite dei 50ms a 20Hz.
*   **Jitter:** La varianza dei tempi tra frame successivi. Un jitter alto può causare scatti nella guida autonoma.
*   **Drop Rate (> 50ms):** La percentuale di frame che superano il budget temporale, portando potenzialmente alla perdita di dati LiDAR (skip).

---

## 3. Visualizzazione (Log Profiler Figures)

I risultati vengono salvati nella cartella `log_profiler/figures/`:
*   **Latency Boxplot:** Distribuzione della latenza totale tra gli algoritmi. Evidenzia gli outliers.
*   **Stacked Bar Chart:** Ripartizione del tempo tra le fasi della pipeline. Permette di identificare istantaneamente il collo di bottiglia (es. se è il clustering o la stima).
*   **Cones Stability:** Grafico del numero di coni rilevati nel tempo. Permette di vedere se l'algoritmo è stabile o se "perde" i coni tra un frame e l'altro.

---

## 4. Come Eseguire un Benchmark

Per avviare una sessione completa di test:
```bash
# Sostituisci il percorso con quello della tua rosbag
./src/fs_lidar_perception/scripts/run_benchmarks.sh /percorso/alla/rosbag

# Una volta terminato, genera i grafici
python3 src/fs_lidar_perception/scripts/plot_metrics.py
```
I risultati saranno disponibili in `log_profiler/figures/`.
