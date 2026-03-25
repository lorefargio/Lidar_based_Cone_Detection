# Clustering (Object Grouping)

Dopo aver rimosso il terreno, la point cloud è composta solo dai punti degli ostacoli. La fase di clustering raggruppa questi punti in "oggetti" candidati a essere coni, prima di passarli all'estimation.

## 1. Algoritmi Implementati

Il progetto utilizza un'architettura modulare (`ClustererInterface`) per testare diverse strategie di raggruppamento:

### 1.1 Grid Clusterer (2D)
Proietta i punti su una griglia 2D (piano XY) e identifica le componenti connesse tra le celle occupate.
*   **Complessità:** $O(N)$ - Richiede una singola iterazione per la voxelizzazione.
*   **Idea:** Veloce e semplice per terreni piatti.
*   **Limiti:** Non considera l'altezza (Z), rischiando di fondere oggetti uno sopra l'altro.

### 1.2 Euclidean Clusterer (Standard)
Utilizza un KD-Tree per trovare punti vicini entro una distanza prefissata (`tolerance`) e raggrupparli ricorsivamente.
*   **Complessità:** $O(N \log N)$ (ricerca nel KD-Tree).
*   **Idea:** Basato sulla distanza euclidea tra punti vicini.
*   **Vantaggi:** Molto preciso nel separare oggetti vicini nello spazio 3D.

### 1.3 String Clusterer
Sfrutta l'ordine radiale/temporale dei punti pubblicati dal driver Hesai. Raggruppa punti consecutivi se la loro distanza è inferiore a una soglia dinamica.
*   **Complessità:** $O(N)$ (lineare).
*   **Idea:** Una stringa di punti vicini nell'ordine di scansione appartiene allo stesso oggetto.
*   **Vantaggi:** L'algoritmo più veloce possibile, non necessita di strutture spaziali pesanti.

### 1.4 DBSCAN & Adaptive DBSCAN
Algoritmo basato sulla densità che raggruppa punti se hanno almeno `min_pts` entro un raggio `epsilon`. La versione Adaptive scala `epsilon` in base alla distanza dal sensore.
*   **Complessità:** $O(N \log N)$.
*   **Idea:** Gli oggetti densi sono cluster, i punti isolati sono rumore (noise).
*   **Vantaggi:** Ottimo per rimuovere il rumore dei sensori e gestire la densità variabile del LiDAR (più denso vicino, più rado lontano).

### 1.5 Voxel Connected Components (3D)
Versione 3D del Grid Clusterer. Crea voxel cubici e cerca componenti connesse tra voxel adiacenti (26-connectivity).
*   **Complessità:** $O(N)$ (con hash map efficiente).
*   **Idea:** Bilancia la velocità della griglia con la precisione spaziale del clustering 3D.
*   **Vantaggi:** Molto robusto a ostacoli sovrapposti (es. coni sotto ponti o ostacoli sospesi).

---

## 2. Parametri e Influenza sul Sistema

| Parametro | Descrizione | Influenza sulla Variazione |
| :--- | :--- | :--- |
| `grid_resolution` | Dimensione della cella (default: 0.20m). | Se troppo grande, i coni vicini si fondono. Se troppo piccola, un singolo cono si divide in più cluster. |
| `dbscan_eps` | Raggio di ricerca per la densità. | Se troppo piccolo, scarta troppi coni (li vede come noise). Se troppo grande, fonde tutto ciò che incontra. |
| `hdbscan_alpha` | Coefficiente di crescita di epsilon con la distanza (Adaptive). | Regola la sensibilità ai coni lontani (meno densi). |
| `dbscan_min_pts` | Punti minimi per formare un cluster (DBSCAN/Adaptive). | Impedisce la creazione di cluster da falsi ritorni o polvere. |
| `min_cluster_size`| Punti minimi totali per un oggetto candidato. | Un cono a 15m può avere solo 5 punti, a 2m ne ha 100. Parametro critico per il richiamo (Recall). |
| `string_max_dist` | Distanza massima radiale tra punti consecutivi. | Parametro core dell'algoritmo String. |


