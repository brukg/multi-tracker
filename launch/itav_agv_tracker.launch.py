from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='itav_agv_tracker',
            executable='itav_agv_tracker',
            name='itav_agv_tracker',
            parameters= [
                # {'clustering_tolerance': 0.15},
                {'map_frame': 'odom'},
                {'clustering_tolerance': 0.5},
                {'min_cluster_size': 2.0},
                {'max_cluster_size': 500.0},
                {'points_threshold': 505},
                # {'points_threshold': 100},
                {'clustering_search_method': 'kdtree'},
                # {'tracker_type': 'kf'},
                {'tracker_type': 'enkf'},
                {'state_size': 4},
                {'control_size': 0},
                {'measurement_size': 4},
                {'ensemble_size': 10000},
                {'predict_num_with_no_update': 25},
                {'max_distance': 0.45},
            ],
            remappings=[
                ("in_1", "scan_matched_points2"),
                ("out", "itav_agv_tracker/point_clusters"),
            ],
        ),

        # Node(
        #     package='itav_agv_tracker',
        #     executable='tracked_objects.py',
        #     name='tracked_objects',
        # )
    ])