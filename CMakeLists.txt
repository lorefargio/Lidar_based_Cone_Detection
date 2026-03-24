cmake_minimum_required(VERSION 3.16)
project(fs_lidar_perception)

# --- 1. BUILD TYPE CONFIGURATION ---
# Default to 'Release' build type for optimized perception performance.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# --- 2. C++ STANDARD ---
# C++17 is recommended for ROS 2 (Galactic/Humble/Jazzy) and enables better optimizations.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 3. COMPILER OPTIMIZATIONS ---
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -O3                 # Maximum optimization level
    -march=native       # Enable CPU-specific instructions (AVX, NEON)
    -mtune=native       # Optimize for the host CPU
    -ffast-math         # Speed up floating point operations
    -DNDEBUG            # Disable assertions for production performance
    -fno-plt            # Optimize shared library calls
  )
endif()

# --- 4. DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Eigen: Required for matrix transformations and linear algebra
find_package(Eigen3 REQUIRED)

# PCL: Point Cloud Library for processing 3D data
find_package(PCL REQUIRED)

# OpenMP: Enables CPU parallelism for significantly improved performance
find_package(OpenMP)

# --- 5. TARGET DEFINITION ---
add_executable(perception_node
  src/node/perception_node.cpp
  src/filtering/bin_based_ground_remover.cpp
  src/filtering/slope_based_ground_remover.cpp
  src/clustering/string_clusterer.cpp
  src/clustering/euclidean_clusterer.cpp
  src/clustering/grid_clusterer.cpp
  src/clustering/dbscan_clusterer.cpp
  src/clustering/adaptive_dbscan_clusterer.cpp
  src/clustering/voxel_connected_components.cpp
  src/estimation/rule_based_estimator.cpp
  src/estimation/model_fitting_estimator.cpp
  src/utils/performance_profiler.cpp
  src/utils/deskewer.cpp
)

# --- 6. INCLUDE DIRECTORIES ---
target_include_directories(perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# --- 7. PCL DEFINITIONS ---
# Required definitions to avoid memory alignment issues between Eigen and PCL
target_compile_definitions(perception_node PUBLIC ${PCL_DEFINITIONS})

# --- 8. LINKING ---
# ROS 2 Dependencies
ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

# External Library Linking
target_link_libraries(perception_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Enable OpenMP if found
if(OpenMP_CXX_FOUND)
  target_link_libraries(perception_node OpenMP::OpenMP_CXX)
endif()

# --- 9. INTERPROCEDURAL OPTIMIZATION (IPO/LTO) ---
# Enables cross-file inlining and advanced linker optimizations.
include(CheckIPOSupported)
check_ipo_supported(RESULT result OUTPUT output)
if(result)
  set_property(TARGET perception_node PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  message(STATUS "IPO/LTO enabled for perception_node")
else()
  message(WARNING "IPO/LTO not supported: ${output}")
endif()

# --- 10. INSTALLATION ---
install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
