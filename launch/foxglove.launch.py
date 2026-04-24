import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. GENERAL ARGUMENTS ---
    bag_arg = DeclareLaunchArgument('bag', default_value='', description='Path to rosbag to play')
    cl_algo_arg = DeclareLaunchArgument('clustering_algorithm', default_value='grid', description='Algorithm: grid, euclidean, depth, dbscan, hdbscan, voxel')
    gr_type_arg = DeclareLaunchArgument('ground_remover_type', default_value='slope_based', description='Algorithm: bin_based, slope_based, patchworkpp')
    
    # --- 2. COMMON PARAMETERS ---
    sensor_z_arg = DeclareLaunchArgument('sensor_z', default_value='-0.50', description='Lidar height from ground')
    max_range_arg = DeclareLaunchArgument('max_range', default_value='25.0', description='Max processing range')
    min_cluster_arg = DeclareLaunchArgument('min_cluster_size', default_value='2', description='Min points per cluster')
    max_cluster_arg = DeclareLaunchArgument('max_cluster_size', default_value='300', description='Max points per cluster')

    # --- 3. GROUND REMOVAL PARAMETERS ---
    # Bin-based
    bin_thr_arg = DeclareLaunchArgument('bin_local_threshold', default_value='0.02')
    bin_cutoff_arg = DeclareLaunchArgument('bin_hard_cutoff', default_value='-0.47')
    # Slope-based
    slope_max_arg = DeclareLaunchArgument('slope_max_slope', default_value='0.08')
    # Patchwork++
    pw_iter_arg = DeclareLaunchArgument('pw_num_iter', default_value='3')
    pw_dist_arg = DeclareLaunchArgument('pw_th_dist', default_value='0.02')
    pw_tgr_arg = DeclareLaunchArgument('pw_enable_TGR', default_value='true')

    # --- 4. CLUSTERING PARAMETERS ---
    # Euclidean
    euc_tol_arg = DeclareLaunchArgument('euclidean_tolerance', default_value='0.35')
    # DBSCAN
    db_eps_arg = DeclareLaunchArgument('dbscan_eps', default_value='0.30')
    # Voxel
    vox_grid_arg = DeclareLaunchArgument('voxel_grid_size', default_value='0.15')
    # Depth-Clustering
    depth_thr_arg = DeclareLaunchArgument('depth_theta_thr', default_value='10.0')
    depth_rings_arg = DeclareLaunchArgument('depth_num_rings', default_value='16')

    # --- 5. ESTIMATION PARAMETERS ---
    pca_lin_arg = DeclareLaunchArgument('pca_max_linearity', default_value='0.8')
    pca_scat_arg = DeclareLaunchArgument('pca_min_scatter', default_value='0.02')
    rule_decay_arg = DeclareLaunchArgument('rule_dynamic_width_decay', default_value='0.005')
    rule_pts_arg = DeclareLaunchArgument('rule_min_points_at_10m', default_value='5')
    log_clusters_arg = DeclareLaunchArgument('log_clusters', default_value='true')
    debug_freq_arg = DeclareLaunchArgument('debug_pub_freq', default_value='10')

    # --- 6. DESKEWING & FILTERS ---
    use_deskew_arg = DeclareLaunchArgument('use_deskewing', default_value='true')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data')
    use_vox_filt_arg = DeclareLaunchArgument('use_voxel_filter', default_value='false')
    vox_size_arg = DeclareLaunchArgument('voxel_size', default_value='0.035')
    static_imu_to_lidar_xyz_arg = DeclareLaunchArgument('static_imu_to_lidar_xyz',default_value='[0.0,0.053,0.0335]')

    # Our perception node
    perception_node = Node(
        package='lidar_perception',
        executable='perception_node',
        name='lidar_perception_node',
        output='screen',
        parameters=[{
            'bag_path': LaunchConfiguration('bag'),
            'clustering_algorithm': LaunchConfiguration('clustering_algorithm'),
            'ground_remover_type': LaunchConfiguration('ground_remover_type'),
            'sensor_z': LaunchConfiguration('sensor_z'),
            'max_range': LaunchConfiguration('max_range'),
            'min_cluster_size': LaunchConfiguration('min_cluster_size'),
            'max_cluster_size': LaunchConfiguration('max_cluster_size'),
            'bin_local_threshold': LaunchConfiguration('bin_local_threshold'),
            'bin_hard_cutoff': LaunchConfiguration('bin_hard_cutoff'),
            'slope_max_slope': LaunchConfiguration('slope_max_slope'),
            'pw_num_iter': LaunchConfiguration('pw_num_iter'),
            'pw_th_dist': LaunchConfiguration('pw_th_dist'),
            'pw_enable_TGR': LaunchConfiguration('pw_enable_TGR'),
            'euclidean_tolerance': LaunchConfiguration('euclidean_tolerance'),
            'dbscan_eps': LaunchConfiguration('dbscan_eps'),
            'voxel_grid_size': LaunchConfiguration('voxel_grid_size'),
            'depth_theta_thr': LaunchConfiguration('depth_theta_thr'),
            'depth_num_rings': LaunchConfiguration('depth_num_rings'),
            'pca_max_linearity': LaunchConfiguration('pca_max_linearity'),
            'pca_min_scatter': LaunchConfiguration('pca_min_scatter'),
            'rule_dynamic_width_decay': LaunchConfiguration('rule_dynamic_width_decay'),
            'rule_min_points_at_10m': LaunchConfiguration('rule_min_points_at_10m'),
            'log_clusters': LaunchConfiguration('log_clusters'),
            'debug_pub_freq': LaunchConfiguration('debug_pub_freq'),
            'use_deskewing': LaunchConfiguration('use_deskewing'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'use_voxel_filter': LaunchConfiguration('use_voxel_filter'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'static_imu_to_lidar_xyz':LaunchConfiguration('static_imu_to_lidar_xyz'),
        }]
    )

    # Foxglove Bridge Node (allows Foxglove Studio to connect via WebSockets)
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': ['/perception/.*','/zed/zed_node/left/image_rect_color', '/tf', '/tf_static', '/rosout', '/lidar_points', '/zed/zed_node/rgb/color/rect/image'],
            'send_buffer_limit': 100000000,
            'use_compression': True,
            'max_update_ms': 50, # 20Hz update rate matches our target
            'min_qos_depth': 1,
        }]
    )

    # Execute a ros2 bag play command if a bag path was provided
    play_bag = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('bag'), "' != ''"])
        ),
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--loop'],
        output='screen'
    )

    return LaunchDescription([
        bag_arg,
        cl_algo_arg,
        gr_type_arg,
        sensor_z_arg,
        max_range_arg,
        min_cluster_arg,
        max_cluster_arg,
        bin_thr_arg,
        bin_cutoff_arg,
        slope_max_arg,
        pw_iter_arg,
        pw_dist_arg,
        pw_tgr_arg,
        euc_tol_arg,
        db_eps_arg,
        vox_grid_arg,
        depth_thr_arg,
        depth_rings_arg,
        pca_lin_arg,
        pca_scat_arg,
        rule_decay_arg,
        rule_pts_arg,
        log_clusters_arg,
        debug_freq_arg,
        use_deskew_arg,
        imu_topic_arg,
        use_vox_filt_arg,
        vox_size_arg,
        static_imu_to_lidar_xyz_arg,
        perception_node,
        foxglove_node,
        play_bag,
    ])
        
