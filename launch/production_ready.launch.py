import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. GENERAL PIPELINE SELECTION ---
    cl_algo_arg = DeclareLaunchArgument('clustering_algorithm', default_value='grid', description='Algorithm: grid, euclidean, depth, dbscan, hdbscan, voxel')
    gr_type_arg = DeclareLaunchArgument('ground_remover_type', default_value='patchworkpp', description='Algorithm: bin_based, slope_based, patchworkpp')
    estimator_type_arg = DeclareLaunchArgument('estimator_type', default_value='rule_based')
    
    # --- 2. COMMON GEOMETRIC PARAMETERS ---
    sensor_z_arg = DeclareLaunchArgument('sensor_z', default_value='-0.52', description='Lidar height from ground')
    max_range_arg = DeclareLaunchArgument('max_range', default_value='25.0', description='Max processing range')
    min_cluster_arg = DeclareLaunchArgument('min_cluster_size', default_value='2', description='Min points per cluster')
    max_cluster_arg = DeclareLaunchArgument('max_cluster_size', default_value='300', description='Max points per cluster')

    # --- 3. GROUND REMOVAL PARAMETERS ---
    # Bin-based
    bin_thr_arg = DeclareLaunchArgument('bin_local_threshold', default_value='0.02')
    bin_cutoff_arg = DeclareLaunchArgument('bin_hard_cutoff', default_value='-0.47')
    bin_seg_arg = DeclareLaunchArgument('bin_segments', default_value='500')
    bin_bins_arg = DeclareLaunchArgument('bin_bins', default_value='500')
    # Slope-based
    slope_max_arg = DeclareLaunchArgument('slope_max_slope', default_value='0.08')
    slope_z_diff_arg = DeclareLaunchArgument('slope_max_z_diff', default_value='0.05')
    slope_init_thr_arg = DeclareLaunchArgument('slope_initial_threshold', default_value='0.05')
    slope_seg_arg = DeclareLaunchArgument('slope_segments', default_value='360')
    # Patchwork++
    pw_iter_arg = DeclareLaunchArgument('pw_num_iter', default_value='3')
    pw_dist_arg = DeclareLaunchArgument('pw_th_dist', default_value='0.02')
    pw_seeds_arg = DeclareLaunchArgument('pw_th_seeds', default_value='0.02')
    pw_dist_v_arg = DeclareLaunchArgument('pw_th_dist_v', default_value='0.1')
    pw_seeds_v_arg = DeclareLaunchArgument('pw_th_seeds_v', default_value='0.02')
    pw_min_r_arg = DeclareLaunchArgument('pw_min_range', default_value='0.5')
    pw_upright_arg = DeclareLaunchArgument('pw_uprightness_thr', default_value='0.707')
    pw_rnr_arg = DeclareLaunchArgument('pw_enable_RNR', default_value='true')
    pw_tgr_arg = DeclareLaunchArgument('pw_enable_TGR', default_value='true')

    # --- 4. CLUSTERING PARAMETERS ---
    # Grid
    grid_res_arg = DeclareLaunchArgument('grid_resolution', default_value='0.12')
    # Euclidean
    euc_tol_arg = DeclareLaunchArgument('euclidean_tolerance', default_value='0.35')
    # DBSCAN
    db_eps_arg = DeclareLaunchArgument('dbscan_eps', default_value='0.30')
    db_min_pts_arg = DeclareLaunchArgument('dbscan_min_pts', default_value='3')
    # HDBSCAN
    hdb_min_pts_arg = DeclareLaunchArgument('hdbscan_min_pts', default_value='5')
    hdb_alpha_arg = DeclareLaunchArgument('hdbscan_alpha', default_value='0.02')
    # Voxel
    vox_grid_arg = DeclareLaunchArgument('voxel_grid_size', default_value='0.02')
    # Depth-Clustering
    depth_thr_arg = DeclareLaunchArgument('depth_theta_thr', default_value='10.0')
    depth_rings_arg = DeclareLaunchArgument('depth_num_rings', default_value='32')

    # --- 5. ESTIMATION PARAMETERS (Strictness Control) ---
    pca_lin_arg = DeclareLaunchArgument('pca_max_linearity', default_value='0.88')
    pca_plan_arg = DeclareLaunchArgument('pca_max_planarity', default_value='0.8')
    pca_scat_arg = DeclareLaunchArgument('pca_min_scatter', default_value='0.02')
    pca_vert_arg = DeclareLaunchArgument('pca_min_verticality', default_value='0.65')
    rule_min_h_arg = DeclareLaunchArgument('rule_min_height', default_value='0.10')
    rule_max_h_arg = DeclareLaunchArgument('rule_max_height', default_value='0.80')
    rule_min_w_arg = DeclareLaunchArgument('rule_base_min_width', default_value='0.10')
    rule_max_w_arg = DeclareLaunchArgument('rule_max_width', default_value='0.50')
    rule_decay_arg = DeclareLaunchArgument('rule_dynamic_width_decay', default_value='0.005')
    rule_pts_arg = DeclareLaunchArgument('rule_min_points_at_10m', default_value='5')
    rule_pts_cap_arg = DeclareLaunchArgument('rule_min_points_cap', default_value='60')
    rule_int_arg = DeclareLaunchArgument('rule_min_intensity', default_value='5.0')

    # --- 6. POST-PROCESSING & DESKEWING ---
    merge_dist_arg = DeclareLaunchArgument('merge_dist', default_value='0.25')
    tracking_match_dist_arg = DeclareLaunchArgument('tracking_match_dist', default_value='0.45')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/zed/zed_node/imu/data')
    deskew_trans_arg = DeclareLaunchArgument('deskew_use_translation', default_value='true')
    roll_deg_arg = DeclareLaunchArgument(
        'roll_deg', default_value='0.0',
        description='Fine-tune roll correction for extrinsic rotation in degrees'
    )
    pitch_deg_arg = DeclareLaunchArgument(
        'pitch_deg', default_value='0.0',
        description='Fine-tune pitch correction for extrinsic rotation in degrees'
    )
    yaw_deg_arg = DeclareLaunchArgument(
        'yaw_deg', default_value='0.0',
        description='Fine-tune yaw correction for extrinsic rotation in degrees'
    )
    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame', default_value='zed_imu_link',
        description='IMU frame ID for deskewing lever arm (typically zed_imu_link)'
    )
    world_up_axis_arg = DeclareLaunchArgument(
        'world_up_axis', default_value='y',
        description='World frame up-axis ("y" or "z")'
    )
    use_vox_filt_arg = DeclareLaunchArgument('use_voxel_filter', default_value='false')
    vox_size_arg = DeclareLaunchArgument('voxel_size', default_value='0.02')

    # --- 7. LOGGING & DIAGNOSTICS ---
    log_dir_arg = DeclareLaunchArgument('log_dir', default_value='log_profiler/')
    log_clusters_arg = DeclareLaunchArgument('log_clusters', default_value='false') # Default false for production
    log_all_clusters_arg = DeclareLaunchArgument('log_all_clusters', default_value='false', description='Log all clusters for recall analysis')
    debug_freq_arg = DeclareLaunchArgument('debug_pub_freq', default_value='50') # Slow down debug publishing in production
    debug_arg = DeclareLaunchArgument('debug', default_value='false', description='Toggle debug visualization topics')

    # LiDAR Perception Node
    perception_node = Node(
        package='lidar_perception',
        executable='perception_node',
        name='lidar_perception_node',
        output='screen',
        parameters=[{
            'clustering_algorithm': LaunchConfiguration('clustering_algorithm'),
            'ground_remover_type': LaunchConfiguration('ground_remover_type'),
            'estimator_type': LaunchConfiguration('estimator_type'),
            'sensor_z': LaunchConfiguration('sensor_z'),
            'max_range': LaunchConfiguration('max_range'),
            'min_cluster_size': LaunchConfiguration('min_cluster_size'),
            'max_cluster_size': LaunchConfiguration('max_cluster_size'),
            'bin_local_threshold': LaunchConfiguration('bin_local_threshold'),
            'bin_hard_cutoff': LaunchConfiguration('bin_hard_cutoff'),
            'bin_segments': LaunchConfiguration('bin_segments'),
            'bin_bins': LaunchConfiguration('bin_bins'),
            'slope_max_slope': LaunchConfiguration('slope_max_slope'),
            'slope_max_z_diff': LaunchConfiguration('slope_max_z_diff'),
            'slope_initial_threshold': LaunchConfiguration('slope_initial_threshold'),
            'slope_segments': LaunchConfiguration('slope_segments'),
            'pw_num_iter': LaunchConfiguration('pw_num_iter'),
            'pw_th_dist': LaunchConfiguration('pw_th_dist'),
            'pw_th_seeds': LaunchConfiguration('pw_th_seeds'),
            'pw_th_dist_v': LaunchConfiguration('pw_th_dist_v'),
            'pw_th_seeds_v': LaunchConfiguration('pw_th_seeds_v'),
            'pw_min_range': LaunchConfiguration('pw_min_range'),
            'pw_uprightness_thr': LaunchConfiguration('pw_uprightness_thr'),
            'pw_enable_RNR': LaunchConfiguration('pw_enable_RNR'),
            'pw_enable_TGR': LaunchConfiguration('pw_enable_TGR'),
            'grid_resolution': LaunchConfiguration('grid_resolution'),
            'euclidean_tolerance': LaunchConfiguration('euclidean_tolerance'),
            'dbscan_eps': LaunchConfiguration('dbscan_eps'),
            'dbscan_min_pts': LaunchConfiguration('dbscan_min_pts'),
            'hdbscan_min_pts': LaunchConfiguration('hdbscan_min_pts'),
            'hdbscan_alpha': LaunchConfiguration('hdbscan_alpha'),
            'voxel_grid_size': LaunchConfiguration('voxel_grid_size'),
            'depth_theta_thr': LaunchConfiguration('depth_theta_thr'),
            'depth_num_rings': LaunchConfiguration('depth_num_rings'),
            'pca_max_linearity': LaunchConfiguration('pca_max_linearity'),
            'pca_max_planarity': LaunchConfiguration('pca_max_planarity'),
            'pca_min_scatter': LaunchConfiguration('pca_min_scatter'),
            'pca_min_verticality': LaunchConfiguration('pca_min_verticality'),
            'rule_min_height': LaunchConfiguration('rule_min_height'),
            'rule_max_height': LaunchConfiguration('rule_max_height'),
            'rule_base_min_width': LaunchConfiguration('rule_base_min_width'),
            'rule_max_width': LaunchConfiguration('rule_max_width'),
            'rule_dynamic_width_decay': LaunchConfiguration('rule_dynamic_width_decay'),
            'rule_min_points_at_10m': LaunchConfiguration('rule_min_points_at_10m'),
            'rule_min_points_cap': LaunchConfiguration('rule_min_points_cap'),
            'rule_min_intensity': LaunchConfiguration('rule_min_intensity'),
            'merge_dist': LaunchConfiguration('merge_dist'),
            'tracking_match_dist': LaunchConfiguration('tracking_match_dist'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'imu_frame': LaunchConfiguration('imu_frame'),
            'world_up_axis': LaunchConfiguration('world_up_axis'),
            'deskew_use_translation': LaunchConfiguration('deskew_use_translation'),
            'roll_deg': LaunchConfiguration('roll_deg'),
            'pitch_deg': LaunchConfiguration('pitch_deg'),
            'yaw_deg': LaunchConfiguration('yaw_deg'),
            'extrinsic_rotation': [0.999743, 0.0226629, 7.2829e-10, 8.06016e-10, -3.42052e-09, -1.0, -0.0226629, 0.999743, -3.43791e-09],
            'extrinsic_translation': [0.0543494, -0.0235914, -0.0488917],
            'use_voxel_filter': LaunchConfiguration('use_voxel_filter'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'log_dir': LaunchConfiguration('log_dir'),
            'log_clusters': LaunchConfiguration('log_clusters'),
            'log_all_clusters': LaunchConfiguration('log_all_clusters'),
            'debug_pub_freq': LaunchConfiguration('debug_pub_freq'),
            'debug': LaunchConfiguration('debug'),
        }]
    )

    return LaunchDescription([
        cl_algo_arg,
        gr_type_arg,
        estimator_type_arg,
        sensor_z_arg,
        max_range_arg,
        min_cluster_arg,
        max_cluster_arg,
        bin_thr_arg,
        bin_cutoff_arg,
        bin_seg_arg,
        bin_bins_arg,
        slope_max_arg,
        slope_z_diff_arg,
        slope_init_thr_arg,
        slope_seg_arg,
        pw_iter_arg,
        pw_dist_arg,
        pw_seeds_arg,
        pw_dist_v_arg,
        pw_seeds_v_arg,
        pw_min_r_arg,
        pw_upright_arg,
        pw_rnr_arg,
        pw_tgr_arg,
        grid_res_arg,
        euc_tol_arg,
        db_eps_arg,
        db_min_pts_arg,
        hdb_min_pts_arg,
        hdb_alpha_arg,
        vox_grid_arg,
        depth_thr_arg,
        depth_rings_arg,
        pca_lin_arg,
        pca_plan_arg,
        pca_scat_arg,
        pca_vert_arg,
        rule_min_h_arg,
        rule_max_h_arg,
        rule_min_w_arg,
        rule_max_w_arg,
        rule_decay_arg,
        rule_pts_arg,
        rule_int_arg,
        imu_topic_arg,
        deskew_trans_arg,
        roll_deg_arg,
        pitch_deg_arg,
        yaw_deg_arg,
        imu_frame_arg,
        world_up_axis_arg,
        use_vox_filt_arg,
        vox_size_arg,
        log_dir_arg,
        log_clusters_arg,
        log_all_clusters_arg,
        debug_freq_arg,
        debug_arg,
        rule_pts_cap_arg,
        merge_dist_arg,
        tracking_match_dist_arg,
        perception_node
    ])
