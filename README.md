# LiDAR-based Cone Detection Node for Formula Student Autonomous

A high-performance, real-time LiDAR processing node designed to detect, classify, and track traffic cones for Formula Student Autonomous racing cars. Optimized for Humble ROS 2, the pipeline leverages CPU parallelization and low-latency C++ structures to operate within a tight 20Hz real-time control budget.

---

## 1. Project Overview

The node receives a raw point cloud from a mechanical rotating LiDAR (e.g. Hesai Pandar 40P) and processes it through a multi-stage sequential pipeline:
1.  **Temporal & Clock Alignment**: Syncs LiDAR absolute sensor clock domains with the host PC and TF tree.
2.  **Spatial Truncation**: Discards out-of-range points (e.g., >25m) and the sensor blind spot (<0.25m).
3.  **High-Precision Deskewing**: Leverages IMU angular velocities and estimated linear velocity to correct point locations for vehicle motion during the rotation sweep.
4.  **Voxel Grid Downsampling**: Filters the deskewed cloud to a 2.0cm voxel grid.
5.  **Ground Segmentation**: Isolates the track surface from candidate obstacle points.
6.  **Spatial Clustering**: Groups obstacle points into distinct volumetric objects.
7.  **Geometric Cone Estimation**: Computes PCA-derived features (linearity, verticality, planarity) to validate the geometric dimensions against regulation Formula Student cones and assign color confidence.
8.  **Asynchronous Active Visualization**: Offloads rendering of markers and debug visualizers to a background worker thread.

---

## 2. Build Dependencies

Before compiling the node, ensure your ROS 2 environment has the following libraries installed:
*   **ROS 2 Distribution**: Humbe (or newer)
*   **Point Cloud Library (PCL)**: `libpcl-dev`
*   **Eigen3**: Linear algebra library (included with ROS 2 perception packages)
*   **OpenMP**: For CPU loop parallelization
*   **patchworkpp**: SOTA ground segmentation library (build from the local workspace directory)
*   **ROS Packages**: `rclcpp`, `rclcpp_components`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `pcl_conversions`, `pcl_ros`, `tf2`, `tf2_ros`, `tf2_eigen`.

---

## 3. Build & Run Instructions

### Step 1: Compile the Workspace
Inside your workspace, run colcon build with release optimizations:
```bash
# Clean build with Release flags
colcon build --packages-select patchworkpp lidar_perception --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

### Step 2: Source the Setup Script
```bash
source install/setup.zsh
```

### Step 3: Launch the Perception Pipeline
Use the launch script to configure parameter overrides (e.g., changing the ground remover type or enabling/disabling deskewing):
```bash
ros 2 launch lidar_perception production_ready.launch.py \
    use_deskewing:=true \
    ground_remover_type:=patchworkpp \
    clustering_algorithm:=grid
```

### Step 4: Run Benchmarks and Analysis (Optional)
To run benchmarks and print out the average and worst-case deskewing distortion resolved on your bag files:
```bash
# Play your bag file and log metrics
ros2 launch lidar_perception foxglove_replay.launch.py \
    profile_name:="my_run" \
    use_deskewing:=true
    
# Once finished, run the Python analysis script to plot results
python3 src/Lidar_based_Cone_Detection/scripts/plot_deskew_metrics.py
```

---

## 4. Documentation Index
Detailed theoretical derivations and implementation notes can be found in:
*   **[Pipeline Architecture](docs/architecture.md)**: Sequential data flow, frame sync, and diagrams.
*   **[Motion Compensation & Deskewing](docs/deskewing.md)**: Physics of rotating LiDAR distortion, lever-arm rotation, and kinematic corrections.
*   **[Ground Segmentation](docs/filtering.md)**: Slope-based, bin-based, and Patchwork++ details.
*   **[Spatial Clustering](docs/clustering.md)**: Grid BFS, DBSCAN, HDBSCAN, Euclidean, Depth-Image, and Voxel CC.
*   **[Geometric Estimation](docs/estimation.md)**: PCA dimensional checks and classification rules.
*   **[Parameters Guide](docs/parameters.md)**: Hyperparameter specifications and configurations.
*   **[Concurrency, Threading & OpenMP](docs/concurrency_and_parallelism.md)**: Multithreaded ROS node wrapper, asynchronous visualization bridge, and OpenMP loop-level optimizations.
