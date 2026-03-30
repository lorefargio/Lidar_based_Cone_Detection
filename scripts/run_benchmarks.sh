#!/bin/bash

# --- Configuration ---
ROSBAG_PATH=$1
RESULTS_DIR="${PERCEPTION_LOG_DIR:-log_profiler}"
ALGORITHMS=("grid" "euclidean" "string" "dbscan" "hdbscan" "voxel")
NODE_NAME="perception_node"

# --- Argument Validation ---
if [ -z "$1" ] || [ ! -d "$1" ]; then
    echo "Error: Invalid or missing rosbag directory path."
    echo "Usage: $0 /path/to/your/rosbag_dir"
    exit 1
fi

# --- Cleanup Function ---
# Gracefully handles interruptions (e.g., Ctrl+C) by stopping all background processes.
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

for ALGO in "${ALGORITHMS[@]}"; do
    echo -e "\nProcessing algorithm: \e[1;34m$ALGO\e[0m"

    # 1. Start Node in a dedicated process group
    # We pass the bag_path parameter which is mandatory for the node's safety check
    setsid ros2 run fs_lidar_perception $NODE_NAME --ros-args \
        -p clustering_algorithm:=$ALGO \
        -p bag_path:="$ROSBAG_PATH" &
    NODE_PID=$!

    # 2. Dynamic wait (Verify the process successfully started)
    sleep 3
    if ! ps -p $NODE_PID > /dev/null; then
        echo "Error: Perception node failed to start for algorithm: $ALGO"
        continue
    fi

    # 3. Play Rosbag
    echo "   Playing rosbag..."
    ros2 bag play "$ROSBAG_PATH" 
    
    # 4. Grace period for final frame processing
    sleep 1

    # 5. Controlled Shutdown (Triggers profiler JSON save)
    echo "   Saving results (Sending SIGINT to node)..."
    kill -INT -$NODE_PID 2>/dev/null

    # Wait with manual timeout to avoid indefinite hanging
    MAX_WAIT=10
    while ps -p $NODE_PID > /dev/null && [ $MAX_WAIT -gt 0 ]; do
        sleep 1
        ((MAX_WAIT--))
    done

    # Force kill if the node does not respond to SIGINT within the timeout
    if ps -p $NODE_PID > /dev/null; then
        echo "   Warning: Node unresponsive; forcing shutdown..."
        kill -9 -$NODE_PID 2>/dev/null
    else
        echo "   Success: Benchmark completed for $ALGO"
    fi
done

echo "=========================================================="
echo "   Benchmarking Session Completed!"
echo "   Results saved in: $RESULTS_DIR"
echo "=========================================================="
