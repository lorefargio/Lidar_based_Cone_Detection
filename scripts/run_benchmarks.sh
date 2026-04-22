#!/bin/bash

# --- Configuration ---
ROSBAG_PATH=$1
RESULTS_DIR="${PERCEPTION_LOG_DIR:-log_profiler}"
NODE_NAME="perception_node"

# 1. Clustering Algorithms to test (Ground Remover: slope_based)
CLUSTERING_ALGOS=("grid" "euclidean" "depth" "dbscan" "hdbscan" "voxel")

# 2. Ground Removal Algorithms to test (Clustering: grid)
GROUND_ALGOS=("bin_based" "slope_based" "patchworkpp")

# --- Argument Validation ---
if [ -z "$1" ] || [ ! -d "$1" ]; then
    echo "Error: Invalid or missing rosbag directory path."
    echo "Usage: $0 /path/to/your/rosbag_dir"
    exit 1
fi

# --- Cleanup Function ---
cleanup() {
    echo -e "\nStop signal received. Terminating all processes..."
    kill -SIGINT -0 2>/dev/null
    exit 1
}
trap cleanup SIGINT SIGTERM

# Prepare output directory
mkdir -p "$RESULTS_DIR"

echo "=========================================================="
echo "   LiDAR Perception Benchmark Session: Starting"
echo "   Bag Path: $ROSBAG_PATH"
echo "=========================================================="

# --- Phase 1: Clustering Benchmarks ---
echo -e "\n>>> PHASE 1: Testing Clustering Algorithms (Ground: patchworkpp)"
for ALGO in "${CLUSTERING_ALGOS[@]}"; do
    echo -e "\nProcessing Clustering: \e[1;34m$ALGO\e[0m"

    setsid ros2 run fs_lidar_perception $NODE_NAME --ros-args \
        -p clustering_algorithm:=$ALGO \
        -p ground_remover_type:=patchworkpp \
        -p bag_path:="$ROSBAG_PATH" &
    NODE_PID=$!

    sleep 3
    if ! ps -p $NODE_PID > /dev/null; then continue; fi

    ros2 bag play "$ROSBAG_PATH" 
    sleep 1

    kill -INT -$NODE_PID 2>/dev/null
    wait $NODE_PID 2>/dev/null
done

# --- Phase 2: Ground Removal Benchmarks ---
echo -e "\n>>> PHASE 2: Testing Ground Removal Algorithms (Clustering: grid)"
for GR_ALGO in "${GROUND_ALGOS[@]}"; do
    # Skip slope_based if already tested in Phase 1 with grid clustering
    if [ "$GR_ALGO" == "slope_based" ]; then
        echo -e "\nSkipping Ground Removal: \e[1;32m$GR_ALGO\e[0m (Already tested in Phase 1 with grid)"
        continue
    fi

    echo -e "\nProcessing Ground Removal: \e[1;32m$GR_ALGO\e[0m"
    
    setsid ros2 run fs_lidar_perception $NODE_NAME --ros-args \
        -p clustering_algorithm:=grid \
        -p ground_remover_type:=$GR_ALGO \
        -p bag_path:="$ROSBAG_PATH" &
    NODE_PID=$!
    
    sleep 3
    if ! ps -p $NODE_PID > /dev/null; then continue; fi

    ros2 bag play "$ROSBAG_PATH" 
    sleep 1

    kill -INT -$NODE_PID 2>/dev/null
    wait $NODE_PID 2>/dev/null
done

echo "=========================================================="
echo "   Benchmarking Session Completed!"
echo "   Results saved in: $RESULTS_DIR"
echo "=========================================================="
