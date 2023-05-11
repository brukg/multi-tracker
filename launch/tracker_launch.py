from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracker',
            executable='tracker',
            name='tracker',
            parameters= [
                {'visualize': True},
                {'publish_rate': 10},
                # {'publish_rate': 20},
                {'map_frame': 'map'},
                {'clustering_tolerance': 0.75},
                # {'clustering_tolerance': 0.50},
                {'min_cluster_size': 1.0},
                # {'min_cluster_size': 2.0},
                {'max_cluster_size': 5000.0},
                {'points_threshold': 500},
                {'max_distance': 2.0},
                {'max_expected_velocity': 5.0},
                {'scanner_range': 10.0},

                # {'cluster_search_method': 'dbscan'},
                {'cluster_search_method': 'kdtree'},
                # {'cluster_search_method': 'meanshift'},
                {'bandwidth': 1.0},
                {'convergence_threshold': 10.0},
                {'max_iterations': 1000},
                # {'tracker_type': 'kf'},
                {'tracker_type': 'enkf'},
                {'sliding_window_size': 15},
                {'state_size': 4},
                {'control_size': 0},
                {'measurement_size': 4},

                # {'data_association_type': 'hungarian'},
                {'data_association_type': 'greedy'},
                # {'data_association_type': 'jpda'},
                # {'data_association_type': 'jcbb'},
                {'ensemble_size': 50},
                {'predict_num_with_no_update': 20},

            ],
            remappings=[
                # ("point_cloud", "scan_matched_points2"),
                # ("point_cloud", "kitti/point_cloud"),

                # ("in_1", "itav_agv/safety_lidar_front_link/scan"),
                # ("in_1", "/safety_lidar_front_link/scan"),
                # ("in_2", "itav_agv/safety_lidar_back_link/scan"),
                # ("in_2", "/safety_lidar_back_link/scan"),
                ("out", "itav_agv/tracker/point_clusters"),
            ],
        ),

        Node(
            package='tracker',
            executable='tracked_objects.py',
            name='scan2pointcloud',
        )
    ])