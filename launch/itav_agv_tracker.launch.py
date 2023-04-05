from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracker',
            executable='tracker',
            name='tracker',
            parameters= [
                # {'clustering_tolerance': 0.15},
                {'publish_rate': 100},
                {'map_frame': 'odom'},
                {'clustering_tolerance': 0.15},
                {'min_cluster_size': 2.0},
                {'max_cluster_size': 50000.0},
                {'points_threshold': 500},
                {'cluster_search_method': 'kdtree'},
                # {'cluster_search_method': 'meanshift'},
                {'bandwidth': 1.0},
                {'convergence_threshold': 1.0},
                {'max_iterations': 100},
                # {'tracker_type': 'kf'},
                {'tracker_type': 'enkf'},
                {'state_size': 4},
                {'control_size': 0},
                {'measurement_size': 4},
                {'ensemble_size': 50},
                {'predict_num_with_no_update': 15},
                {'max_distance': 1.0},
            ],
            remappings=[
                ("in_1", "scan_matched_points2"),
                ("out", "tracker/point_clusters"),
            ],
        ),

        # Node(
        #     package='tracker',
        #     executable='tracked_objects.py',
        #     name='tracked_objects',
        # )
    ])