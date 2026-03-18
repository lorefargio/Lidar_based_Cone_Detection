# Estimation (Cone Classification)

L'ultima fase della nostra pipeline riceve i cluster di punti (oggetti candidati) e decide quali sono effettivamente coni, calcolandone la posizione (XYZ) e il colore. Qui la precisione è critica per evitare che l'auto sterzi verso un marciapiede scambiandolo per un cono.

## 1. Algoritmi Implementati

Il progetto utilizza l'interfaccia `EstimatorInterface` per implementare diverse strategie di classificazione:

### 1.1 Rule-Based Estimator (Avanzato)
Usa regole geometriche e statistiche sui cluster per validare la loro forma.
*   **Complessità:** $O(N)$ (lineare).
*   **Idea:** Un cono ha dimensioni specifiche (circa 22.8cm di base e 32.5cm di altezza). Verifichiamo il bounding box (larghezza, altezza) e il rapporto tra i due (Aspect Ratio).
*   **Evoluzione (Soglie Dinamiche):** Poiché a 15m un cono ha meno punti e sembra più piccolo, le soglie scalano in base alla distanza radiale ($r$) dal sensore.
*   **PCA Integrated:** Utilizza l'Analisi delle Componenti Principali per scartare oggetti lineari (paletti, gambe) o piatti (muri).

### 1.2 Model Fitting Estimator (RANSAC)
Cerca di fittare un modello geometrico noto (Cilindro o Cono) sui punti del cluster.
*   **Complessità:** $O(I \cdot N)$ (dove $I$ sono le iterazioni di RANSAC).
*   **Idea:** Un oggetto è un cono se un'alta percentuale dei suoi punti (inliers) cade sulla superficie di un cilindro di raggio noto (circa 11.4cm).
*   **Vantaggi:** Molto robusto ai cluster sporchi (punti di terreno residui o rumore).
*   **Limiti:** Più lento del Rule-Based, richiede molti punti per un fitting affidabile.

---

## 2. Analisi PCA (Principal Component Analysis)

La PCA è la nostra arma "state-of-the-art" per la classificazione leggera. Calcoliamo gli autovalori ($\lambda_1, \lambda_2, \lambda_3$) della matrice di covarianza del cluster:

1.  **Linearity ($L$):** $(\lambda_1 - \lambda_2) / \lambda_1$. Se $L > 0.8$, l'oggetto è una linea (es. gamba o paletto).
2.  **Planarity ($P$):** $(\lambda_2 - \lambda_3) / \lambda_1$. Se $P > 0.8$, l'oggetto è un piano (es. pezzo di muro).
3.  **Scatter ($S$):** $\lambda_3 / \lambda_1$. Se $S$ è bilanciato, l'oggetto è volumetrico (un vero cono ha $S > 0.05$).

---

## 3. Parametri e Influenza sul Sistema

| Parametro | Descrizione | Influenza sulla Variazione |
| :--- | :--- | :--- |
| `min_height` / `max_height` | Limiti in Z del cono (0.15m - 0.35m). | Fondamentale per scartare piccoli detriti o persone alte. |
| `min_points_at_10m` | Punti minimi attesi a 10 metri. | Parametro chiave per il richiamo (Recall). Se troppo alto, i coni lontani spariscono. |
| `dynamic_width_decay` | Riduzione della larghezza minima con $r$. | Compensa la divergenza dei raggi del LiDAR a lunga distanza. |
| `pca_max_linearity` | Soglia per scartare oggetti lineari. | Più bassa è, più rigido è lo scarto delle gambe o paletti. |
| `pca_min_scatter` | Soglia per assicurare tridimensionalità. | Se troppo alta, scarta coni visti da pochi angoli (es. coni laterali). |
| `min_inlier_ratio` | (RANSAC) % punti sul cilindro. | Definisce la confidenza del fitting geometrico. |

---

## 4. Confronto Tecnico

| Algoritmo | Velocità | Precisione | Robustezza Rumore |
| :--- | :--- | :--- | :--- |
| **Rule-Based** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| **Rule-Based + PCA** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Model Fitting (RANSAC)** | ⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
