#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters

import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
# import tf listener
import tf2_ros

from rclpy.qos import qos_profile_sensor_data
class TrackedObjects:
    def __init__(self, node):
        self.node = node
        self.tracked_objects = []
        self.timestamp = None
        # set qos profile reliability


        self.qos_profile = qos_profile_sensor_data

        self.tracked_objects_sub = node.create_subscription(Float32MultiArray, 'obstacles_state', self.tracked_objects_callback, qos_profile=self.qos_profile)
        self.laser_front_sub = node.create_subscription(LaserScan, '/safety_lidar_front_link/scan', self.laser_callback, qos_profile_sensor_data)
        self.laser_back_sub = node.create_subscription(LaserScan, '/safety_lidar_back_link/scan', self.laser_callback, qos_profile_sensor_data)
        
        # self.a = message_filters.Subscriber(self.node, LaserScan, 'itav_agv/safety_lidar_front_link/scan')
        # self.b =  message_filters.Subscriber(self.node, LaserScan, 'itav_agv/safety_lidar_back_link/scan')

        
        # self.a = message_filters.Subscriber(self.node, LaserScan, 'safety_lidar_front_link/scan')
        # self.b =  message_filters.Subscriber(self.node, LaserScan, 'safety_lidar_back_link/scan')


        # self.ts = message_filters.ApproximateTimeSynchronizer([self.a, self.b], 1, 1)
        # self.ts.registerCallback(self.laser_callback)
        
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        # self.point_cloud_transformer = PointCloud2Transformer(self.tf_buffer, 'map')

        self.point2_pub = node.create_publisher(PointCloud2, 'scan_matched_points2', 1)
        self.tracked_objects_sub  # prevent unused variable warning
        # self.plt_timer = node.create_timer(0.025, self.plot_tracked_objects)
    
    
    # def laser_callback(self, msg, msg2):
    def laser_callback(self, msg):
        # print("laser callback")
        # convert laser scan to point cloud
        lp = lg.LaserProjection()
        # remove points that are are too close to the robot as false positives using laser_geometry
        # pc = lp.projectLaser(msg, min_range=0.1)
        

        pc = lp.projectLaser(msg)
        
        # transform point cloud to map frame
        # transform = self.tf_buffer.lookup_transform(
        #         'odom',  # target frame
        #         msg.header.frame_id,  # source frame
        #         rclpy.time.Time(),
        #         rclpy.duration.Duration(seconds=10)  # timeout
        #     )
        # # pc = self.point_cloud_transformer.transform(msg, transform)
        # pc = do_transform_cloud(pc, transform)

        # publish point cloud
        self.point2_pub.publish(pc)
        # pc = lp.projectLaser(msg2)
        # self.point2_pub.publish(pc)
        



    
    def tracked_objects_callback(self, msg):
        self.tracked_objects = np.array(msg.data).reshape(-1, 4)


    def plot_tracked_objects(self):
        x = range(0, len(self.tracked_objects))
        # print("tracked objects: ", self.tracked_objects)
        if len(self.tracked_objects) > 0:
            for i in range(0, len(self.tracked_objects)):
                plt.plot(self.timestamp, self.tracked_objects[i, 0], 'ro')
                plt.plot(self.timestamp, self.tracked_objects[i, 1], 'bo')
                plt.plot(self.timestamp, self.tracked_objects[i, 2], 'go')
                plt.plot(self.timestamp, self.tracked_objects[i, 3], 'yo')
            # plt.scatter(self.tracked_objects[:, 0], self.tracked_objects[:, 1])
            # plt.legend(['xy'])
            # plt.plot(x, self.tracked_objects[:, 2], 'g')
            # plt.plot(x, self.tracked_objects[:, 3], 'y')
            plt.legend(['vx','vy'])
            plt.pause(0.00005)
        
        # plt.show()

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('tracked_objects')

    tracked_objects = TrackedObjects(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
