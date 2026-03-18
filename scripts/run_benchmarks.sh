#!/bin/bash

# --- Configurazione ---
ROSBAG_PATH=$1
# Usa il percorso assoluto basato sulla home dell'utente corrente
RESULTS_DIR="${HOME}/lidar_ws/log_profiler"
ALGORITHMS=("grid" "euclidean" "string" "dbscan" "hdbscan" "voxel")
NODE_NAME="perception_node"

# --- Controllo Argomenti ---
if [ -z "$1" ] || [ ! -d "$1" ]; then
    echo "❌ Errore: Percorso rosbag non valido o non specificato."
    echo "Uso: $0 /percorso/del/tuo/rosbag_dir"
    exit 1
fi

# --- Funzione di Pulizia Globale ---
cleanup() {
    echo -e "\n🛑 Ricevuto segnale di stop. Terminazione di tutti i processi..."
    # Killiamo tutti i processi che appartengono a questo gruppo (grazie a 0)
    kill -SIGINT -0 2>/dev/null
    exit 1
}
trap cleanup SIGINT SIGTERM

# Preparazione ambiente
mkdir -p "$RESULTS_DIR"

echo "==========================================="
echo "   Benchmark Session: Starting"
echo "   Bag: $ROSBAG_PATH"
echo "==========================================="

for ALGO in "${ALGORITHMS[@]}"; do
    echo -e "\n🔸 Algoritmo: \e[1;34m$ALGO\e[0m"

    # 1. Avvio Nodo con Process Group dedicato
    # Usiamo 'setsid' per far sì che il nodo sia a capo di un suo gruppo
    setsid ros2 run fs_lidar_perception $NODE_NAME --ros-args -p clustering_algorithm:=$ALGO > /dev/null 2>&1 &
    NODE_PID=$!

    # 2. Attesa dinamica (Verifica se il processo è attivo)
    sleep 3
    if ! ps -p $NODE_PID > /dev/null; then
        echo "❌ Errore: Il nodo non è partito correttamente."
        continue
    fi

    # 3. Esecuzione Bag
    echo "   ▶️  Playing rosbag..."
    ros2 bag play "$ROSBAG_PATH" 
    
    # 4. Grace period
    sleep 1

    # 5. Shutdown Controllato
    echo "   💾 Saving results (Sending SIGINT)..."
    # Inviamo il segnale al gruppo di processi (segno meno)
    kill -INT -$NODE_PID 2>/dev/null

    # Wait con timeout manuale per evitare il blocco infinito
    MAX_WAIT=10
    while ps -p $NODE_PID > /dev/null && [ $MAX_WAIT -gt 0 ]; do
        sleep 1
        ((MAX_WAIT--))
    done

    # Se ancora vivo dopo il timeout, forza il kill
    if ps -p $NODE_PID > /dev/null; then
        echo "   ⚠️  Il nodo non ha risposto, forzo la chiusura..."
        kill -9 -$NODE_PID 2>/dev/null
    else
        echo "   ✅ Test $ALGO completato."
    fi
done

echo "==========================================="
echo "   ✨ Benchmarking Terminato!"
echo "   Risultati in: $RESULTS_DIR"
echo "==========================================="