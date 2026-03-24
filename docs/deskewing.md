# LiDAR Deskewing Module

## 1. Il Problema: Motion Distortion
Nelle competizioni Formula Student, il veicolo si muove a velocità elevate (fino a 100+ km/h). Un LiDAR rotativo (come l'Hesai Pandar40P) non acquisisce tutti i punti di un frame nello stesso istante: effettua una scansione a 360° che dura circa 50-100ms (10-20Hz).

Se l'auto ruota o trasla durante questa scansione, la nuvola di punti risultante appare "distorta" (skewed). Ad esempio, un cono perfettamente circolare potrebbe apparire allungato o sfasato, portando a errori nella stima della posizione e della forma (PCA fallisce).

## 2. La Soluzione: Deskewing
Il modulo `Deskewer` corregge questa distorsione utilizzando i dati ad alta frequenza (400Hz) dell'IMU della ZED.

### 2.1 Interpolazione Temporale (Slerp)
Il LiDAR fornisce un timestamp preciso per **ogni singolo punto** (`PointXYZIRT.timestamp`). Poiché i messaggi IMU arrivano in momenti diversi rispetto ai punti LiDAR, utilizziamo la **Spherical Linear Interpolation (Slerp)** per stimare l'orientamento esatto del veicolo all'istante di acquisizione di ogni punto.

$$q_{interp} = Slerp(q_{before}, q_{after}, \alpha)$$
dove $\alpha$ è il fattore di interpolazione temporale.

### 2.2 Trasformazione Relativa
Invece di trasformare i punti in un frame globale (che introdurrebbe drift), calcoliamo il movimento relativo del veicolo rispetto all'inizio della scansione del LiDAR ($t_{start}$).

1.  Determiniamo l'orientamento all'inizio del frame: $q_{start}$.
2.  Per ogni punto al tempo $t_p$:
    *   Otteniamo $q_p$ tramite interpolazione.
    *   Calcoliamo la rotazione relativa: $\Delta q = q_{start}^{-1} \cdot q_p$.
    *   Applichiamo la rotazione al punto: $P_{corrected} = \Delta q \cdot P_{raw}$.

## 3. Integrazione nel Sistema
Il modulo è implementato come una classe utility utilizzata dal `LidarPerceptionNode`.
*   **Input:** Messaggi `sensor_msgs/msg/Imu` (bufferizzati in una coda).
*   **Processo:** Viene chiamato immediatamente dopo la conversione del messaggio PointCloud2, prima di qualsiasi fase di filtraggio o clustering.
*   **Performance:** Utilizza Eigen per operazioni vettoriali ottimizzate e un buffer thread-safe.

## 4. Requisiti Hardware
*   LiDAR con supporto ai timestamp per punto.
*   IMU a bassa latenza.
