import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Argument for the rosbag file path
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value='',
        description='Path to the rosbag to play. If empty, the bag will not be played.'
    )

    clusterer_arg = DeclareLaunchArgument(
        'clustering_algorithm',
        default_value='grid',
        description='Clustering algorithm to use: "grid", "string", or "euclidean" , "dbscan" , "hdbscan", "voxel"'
    )

    ground_remover_arg = DeclareLaunchArgument(
        'ground_remover_type',
        default_value='slope_based',
        description='Ground removal algorithm to use: "bin_based" or "slope_based"'
    )

    estimator_arg = DeclareLaunchArgument(
        'estimator_type',
        default_value='rule_based',
        description='Estimator algorithm to use: "rule_based" or "ransac"'
    )

    dynamic_width_decay_arg = DeclareLaunchArgument(
        'dynamic_width_decay',
        default_value='0.005',
        description='Decay factor for dynamic width threshold in rule-based estimator'
    )

    min_points_at_10m_arg = DeclareLaunchArgument(
        'min_points_at_10m',
        default_value='10',
        description='Minimum points expected for a cone at 10 meters distance'
    )

    pca_max_linearity_arg = DeclareLaunchArgument(
        'pca_max_linearity',
        default_value='0.8',
        description='PCA Max Linearity threshold to reject posts/legs'
    )

    pca_min_scatter_arg = DeclareLaunchArgument(
        'pca_min_scatter',
        default_value='0.05',
        description='PCA Min Scatter threshold to ensure volumetric shape'
    )

    use_voxel_filter_arg = DeclareLaunchArgument(
        'use_voxel_filter',
        default_value='false',
        description='Whether to use voxel grid downsampling'
    )

    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='Leaf size for voxel grid filter (meters)'
    )

    # Our perception node
    perception_node = Node(
        package='fs_lidar_perception',
        executable='perception_node',
        name='lidar_perception_node',
        output='screen',
        parameters=[{
            'clustering_algorithm': LaunchConfiguration('clustering_algorithm'),
            'ground_remover_type': LaunchConfiguration('ground_remover_type'),
            'estimator_type': LaunchConfiguration('estimator_type'),
            'dynamic_width_decay': LaunchConfiguration('dynamic_width_decay'),
            'min_points_at_10m': LaunchConfiguration('min_points_at_10m'),
            'pca_max_linearity': LaunchConfiguration('pca_max_linearity'),
            'pca_min_scatter': LaunchConfiguration('pca_min_scatter'),
            'use_voxel_filter': LaunchConfiguration('use_voxel_filter'),
            'voxel_size': LaunchConfiguration('voxel_size')
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
            'topic_whitelist': ['.*'],
            'send_buffer_limit': 100000000,
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
        clusterer_arg,
        ground_remover_arg,
        estimator_arg,
        dynamic_width_decay_arg,
                min_points_at_10m_arg,
                pca_max_linearity_arg,
                pca_min_scatter_arg,
                use_voxel_filter_arg,
                voxel_size_arg,
                perception_node,
                foxglove_node,
                play_bag
            ])
        