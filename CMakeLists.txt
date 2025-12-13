cmake_minimum_required(VERSION 3.8)
project(fs_lidar_perception)

# Opzioni compilatore
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Eseguibile unico
add_executable(perception_node
  src/perception_node.cpp
  src/ground_remover.cpp
  src/string_clusterer.cpp
  src/cone_estimator.cpp
)

# --- CORREZIONE CRITICA QUI SOTTO ---
# Usa ament_target_dependencies per i pacchetti ROS.
# Questo risolve l'errore "No such file" per visualization_msgs.
ament_target_dependencies(perception_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

# Linka PCL manualmente (non è un pacchetto ROS)
target_link_libraries(perception_node
  ${PCL_LIBRARIES}
)

# Installazione
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})

ament_package()
