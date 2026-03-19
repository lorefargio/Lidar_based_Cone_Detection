# Camera-LiDAR Fusion Integration

Questo documento descrive le feature della pipeline di percezione per supportare la fusione sensoriale (Camera-LiDAR), migliorando la visualizzazione e l'output dei dati.

## 1. Obiettivi della Modifica
L'integrazione mira a fornire al modulo di fusione tutte le informazioni necessarie per validare i rilevamenti:
*   **Visualizzazione Distanza:** Visualizzazione in tempo reale della distanza del sensore da ogni cono.
*   **Range & Bearing:** Calcolo esplicito della distanza radiale ($r$) e dell'angolo ($	theta$) del centroide del cluster.
*   **Full Point Cloud:** Output di tutti i punti appartenenti a ciascun cluster identificato come cono, utile per il controllo di inclusione nelle bounding box 2D/3D della camera.

---

## 2. Dettagli Implementativi

### 2.1 Struttura Dati (`types.hpp`)
La struttura `Cone` è stata estesa per includere i metadati di fusione:
```cpp
struct Cone {
    float x, y, z;        // Posizione cartesiana
    float range;          // Distanza radiale (m)
    float bearing;        // Angolo (rad)
    // ...
    PointCloudPtr cloud;  // Puntatore ai punti originali del cluster
};
```

### 2.2 Output del Sistema
Il nodo pubblica ora due flussi di dati principali per la fusione:
1.  **`/perception/cones`**: Pubblica i centroidi dei coni. L'intensità del punto viene usata per trasportare il valore del `range` per una rapida consultazione.
2.  **`/perception/cone_points`**: Pubblica l'unione di tutti i punti dei cluster validati. Questo permette di proiettare i punti LiDAR sull'immagine della camera e verificare se cadono all'interno della bounding box rilevata dalla rete neurale.

---

## 3. Visualizzazione con Foxglove
È stata aggiunta una nuova namespace di marker: `distance_labels`.
*   **Tipo:** `TEXT_VIEW_FACING`
*   **Contenuto:** Distanza in metri con precisione al centimetro (es. "12.45m").
*   **Posizionamento:** Automaticamente sopra il cilindro di visualizzazione del cono.

---

