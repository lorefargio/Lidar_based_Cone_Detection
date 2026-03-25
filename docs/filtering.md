# Filtering (Ground Removal)

La fase di filtering è il primo stadio della nostra pipeline di percezione. Il suo scopo principale è separare i punti appartenenti al terreno da quelli che rappresentano ostacoli (come i coni). In questa fase, la precisione è fondamentale: rimuovere troppi punti può "tagliare" la base del cono rendendo difficile il clustering, mentre rimuoverne troppo pochi può generare falsi ostacoli.

## 1. Algoritmi Implementati

Il progetto utilizza un'architettura modulare (`GroundRemoverInterface`) che permette di scegliere tra due algoritmi principali:

### 1.1 Bin-Based Ground Remover (Classico)
Divide la nuvola in una griglia polare (settori angolari e bin radiali). Per ogni bin, identifica il punto con l'altezza minima ($Z_{min}$) e considera terreno tutti i punti nel bin che non superano una soglia fissa rispetto a $Z_{min}$.
*   **Complessità:** $O(N)$ - Richiede una singola iterazione sui punti.
*   **Idea:** Basata sull'assunzione che il terreno sia localmente piatto all'interno di un piccolo bin.
*   **Limiti:** Sensibile a rumore (outlier sotto il terreno) e non gestisce bene le pendenze.

### 1.2 Slope-Based Ground Remover
Analizza i punti lungo i canali radiali. Calcola la pendenza (slope) tra punti consecutivi in ordine di distanza dal sensore.
*   **Complessità:** $O(N \log N)$ (a causa dell'ordinamento radiale dei settori).
*   **Idea:** Se la pendenza tra due punti adiacenti è piccola, entrambi appartengono al suolo. Se c'è un salto brusco in $Z$, il punto più lontano è un ostacolo.
*   **Vantaggi:** Molto più preciso nel preservare la base dei coni, poiché riconosce la variazione improvvisa di pendenza tra asfalto e cono.


---

## 2. Parametri e Influenza sul Sistema

Tutti i parametri sono configurabili tramite file di launch:

| Parametro | Descrizione | Influenza sulla Variazione |
| :--- | :--- | :--- |
| `sensor_z` | Altezza del LiDAR dal suolo (default: -0.52m). | Se errato, l'intera pipeline fallisce ignorando o tagliando i coni. |
| `slope_max_slope` | (Slope-based) Pendenza massima del suolo. | Aumentarlo permette di rilevare terreni più ripidi, ma rischia di includere basi di oggetti. |
| `bin_local_threshold` | (Bin-based) Altezza minima sopra il punto più basso. | Troppo alta: si perdono piccoli ostacoli. Troppo bassa: la polvere o il rumore diventano ostacoli. |
| `pw_th_dist` | (Patchwork++) Soglia di distanza dal piano. | Definisce la precisione del fit del terreno. |

---

## 3. Confronto Tecnico

| Algoritmo | Velocità | Robustezza | Casi d'Uso |
| :--- | :--- | :--- | :--- |
| **Bin-Based** | Test rapidi, terreni perfettamente piatti. |
| **Slope-Based**| Setup standard, necessità di preservare basi dei coni. |
