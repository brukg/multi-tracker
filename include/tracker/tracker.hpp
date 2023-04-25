#ifndef TRACKER_NODE__TRACKER_NODE_NODE_HPP_
#define TRACKER_NODE__TRACKER_NODE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>

#include "omp.h"


#include <dlib/optimization/max_cost_assignment.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"



#include <pcl_ros/transforms.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h> //replace with intel's kdtree
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "dbscan/dbScan.h"

#include "tracker/kf.hpp"


namespace tracker
{
  /**
   * @class tracker::TrackerNode
   * @brief Receives Pointcloud2 message from lidar sensor and filter its points with an optional pcl filter.
   * 
   */
  class TrackerNode : public rclcpp::Node
  {
    public:
      
      /**
       * @brief A constructor for
       * @param options Additional options to control creation of the node.
       */
      explicit TrackerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
      
      /**
       * @brief A destructor for 
       */
      ~TrackerNode() {};

    protected:
      /**
       * @brief Use a node filter of pcl library
       * @param msg Pointcloud2 message receveived from the ros2 node
       * @return -
       * @details Omit pointcloud filtering in this example
       */
      void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr msg2);

      /**
       * @brief point cloud callback
       * 
       */
      void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

      /**
       * @brief cluster the point clouds
        * 
        */
      void clusterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std_msgs::msg::Header header);
      /**
       * @brief Track the clusters of points
       * 
       */
      void trackObjects();
      // void trackObjects(std::vector<Eigen::VectorXd> &meas);

      /**
       * @brief publish markers and velocity of the objects
       * 
       */
      void publishMarkers();

      /**
       * @brief cluster the point clouds and return the cluster points
       * 
       */
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> meanShiftCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      
      /**
       * @brief associate the objects with the measurements
       * 
       */
       void associateVectors(std::vector<Eigen::VectorXd>& pred, std::vector<Eigen::VectorXd>& meas);

      /**
       * @brief associate the objects with the measurements using the JPDA algorithm
       * 
       */      
      std::vector<int> JPDAAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold);

      /**
       * @brief associate the objects with the measurements using the greedy algorithm
       * 
       */
      std::vector<int> GreedyAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold);

      /**
       * @brief associate the objects with the measurements using the Hungarian algorithm
       * @param predictions
       * @param measurements
       * 
       */
      std::vector<int> HungarianAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold);
      
      /**
       * @brief check if the predicted and measured points are compatible
       * 
       * @param pred 
       * @param meas 
       * @param gate_threshold 
       * @return true 
       * @return false 
       */
      bool isCompatible(const Eigen::VectorXd& pred, const Eigen::VectorXd& meas, double gate_threshold);

      /**
       * @brief associate the objects with the measurements using the JCBB algorithm
       * 
       * @param predictions
        * @param measurements
        * 
        */
      std::vector<int> JCBBAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold); 

      /**
       * @brief associate the objects with the measurements using the JCBB algorithm
       * 
       * @param predictions 
       * @param measurements 
       * @param gate_threshold 
       * @param pred_idx 
       * @param current_association 
       * @param best_association 
       */
      void JCBB(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold, size_t pred_idx, std::vector<int>& current_association, std::vector<int>& best_association); 

      /**
       * @brief Construct a new euclidean distance object
       * 
       * @param p1 
       * @param p2 
       */
      float euclideanDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

      /**
       * @brief Construct a new kernel mean shift object
       * 
       * @param point 
       * @param cloud 
       * @param bandwidth 
       */
      pcl::PointXYZ kernel_mean_shift(const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
      /**
       * @brief Construct a new kernel mean shift object
       * 
       * @param point 
       * @param cloud 
       * @param bandwidth 
       */
      std::vector<Eigen::Vector2f> compute_xy_centroids(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);






      // ROS2 subscriber and related topic name
      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
      message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_1_;
      message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_2_;

      using approximate_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;
      std::unique_ptr<message_filters::Synchronizer<approximate_policy>> sync_;
      laser_geometry::LaserProjection projector_1, projector_2;
      // obejects id publisher
      rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr objects_id_pub_;
      // objects size publisher
      rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr state_size_pub_;
      
      rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr objects_state_pub_; // publish objects x,y,vx,vy
      
       
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // visualization marker publisher
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_pub_; // visualization marker publisher
      
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // path publisher for the objects trajectory
      
      // rclcpp::Publisher<visualization_msgs::msg::Marker::POINTS>::SharedPtr point_particles_pub_; // publish the particles of the objects
      pcl::PointCloud<pcl::PointXYZ>::Ptr old_cloud, temp;// (new pcl::PointCloud<pcl::PointXYZ>);
      

      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      // timer 
      rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::string points2_1_sub_, points2_2_sub_;
      
      // clustering parameters
      double clustering_tolerance_, min_cluster_size_, max_cluster_size_;
      double bandwidth_, convergence_threshold_;
      int max_iterations_, points_threshold_;
      double dt_, t_old_, t_new_;
      double range_;
      bool first_scan_ , is_update_, visualise_;
      std::string cluster_search_method_;
      
      // ROS2 publisher and related topic name 
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
      std::string param_topic_pointcloud_out;

      // std::vector<KalmanFilter> objects_;
      std::vector<EnsembleKalmanFilter> objects_enkf;
      std::vector<EnsembleKalmanFilter> objects_;
      
      int n_, m_, c_, N_, predict_num_;
      double max_dist_, max_expected_velocity_;
      int publish_rate_;
      std::vector<Eigen::VectorXd> meas;
      int id_;
      std::string tracker_type_, association_type_;
      std::string map_frame_, robot_base_frame_;

      visualization_msgs::msg::MarkerArray centroid_array;
      visualization_msgs::msg::Marker centroid_marker;
      // rviz_visual_tools::RvizVisualTools rviz_interface;

  };
  
} // namespace tracker

#endif //TRACKER_NODE__TRACKER_NODE_NODE_HPP_