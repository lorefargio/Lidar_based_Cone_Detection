cmake_minimum_required(VERSION 3.16) # Aggiornato per supportare meglio LTO e policy moderne
project(fs_lidar_perception)

# --- 1. CONFIGURAZIONE BUILD TYPE ---
# Forza la build in 'Release' se non specificato altrimenti.
# Le build di default o Debug sono troppo lente per la percezione lidar.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# --- 2. STANDARD C++ ---
# C++17 è raccomandato per ROS 2 (Galactic/Humble/Jazzy) e permette ottimizzazioni migliori.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 3. OTTIMIZZAZIONI COMPILATORE ---
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -O3                 # Massima ottimizzazione
    -march=native       # Fondamentale: usa istruzioni specifiche della CPU (AVX, NEON)
    -mtune=native       # Ottimizza per la CPU corrente
    -ffast-math         # Velocizza i calcoli float (utile per distanze e trignometria)
    -DNDEBUG            # Disabilita assert (rimuove controlli di sicurezza extra di Eigen/STL)
    -fno-plt            # Ottimizzazione chiamate funzioni librerie condivise
  )
endif()

# --- 4. DIPENDENZE ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Eigen: Fondamentale per le trasformazioni matriciali
find_package(Eigen3 REQUIRED)

# PCL: Cerca componenti specifici se possibile, o REQUIRED generale
find_package(PCL REQUIRED)

# OpenMP: PCL lo usa internamente. Se presente, migliora drasticamente le performance su multicore.
find_package(OpenMP)

# --- 5. DEFINIZIONE TARGET ---
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
)

# --- 6. INCLUDE DIRECTORIES ---
target_include_directories(perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
# --- 7. DEFINIZIONI PCL ---
# Aggiunge definizioni necessarie per evitare problemi di allineamento memoria con Eigen/PCL
target_compile_definitions(perception_node PUBLIC ${PCL_DEFINITIONS})

# --- 8. LINKING ---
# ROS 2 Dependencies
ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

# Link Librerie Esterne
target_link_libraries(perception_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Abilita OpenMP se trovato (Parallelismo CPU)
if(OpenMP_CXX_FOUND)
  target_link_libraries(perception_node OpenMP::OpenMP_CXX)
endif()

# --- 9. LINK TIME OPTIMIZATION (IPO/LTO) ---
# Questa è l'ottimizzazione "segreta" più potente.
# Permette al linker di fare inlining tra diversi file .cpp
include(CheckIPOSupported)
check_ipo_supported(RESULT result OUTPUT output)
if(result)
  set_property(TARGET perception_node PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  message(STATUS "IPO/LTO enabled per perception_node")
else()
  message(WARNING "IPO/LTO not supported: ${output}")
endif()

# --- 10. INSTALLAZIONE ---
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