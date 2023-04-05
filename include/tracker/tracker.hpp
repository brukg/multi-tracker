#ifndef TRACKER_NODE__TRACKER_NODE_NODE_HPP_
#define TRACKER_NODE__TRACKER_NODE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <dlib/optimization/max_cost_assignment.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h> //replace with intel's kdtree
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/dbscan.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <tracker/kf.hpp>


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
       * @brief Use a no filter of pcl library
       * @param msg Pointcloud2 message receveived from the ros2 node
       * @return -
       * @details Omit pointcloud filtering in this example
       */
      void topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

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
       * @brief Construct a new euclidean distance object
       * 
       * @param p1 
       * @param p2 
       */
      float euclidean_distance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

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
      // obejects id publisher
      rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr objects_id_pub_;
      // objects size publisher
      rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr state_size_pub_;
      // publish objects x,y,vx,vy
      rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr objects_state_pub_;
      // visualization marker publisher 
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centroid_pub_;
      // path publisher for the objects trajectory
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
      // visualize the objects velocity
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr velocity_pub_;

      // timer 
      rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
      rclcpp::TimerBase::SharedPtr timer_;
      std::string points2_1_sub_, points2_2_sub_;
      
      // clustering parameters
      double clustering_tolerance_, min_cluster_size_, max_cluster_size_;
      double bandwidth_, convergence_threshold_;
      int max_iterations_;
      int points_threshold_;
      double dt_, t_old_, t_new_;
      bool first_scan_ , is_update_;
      std::string cluster_search_method_;
      // ROS2 publisher and related topic name 
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
      std::string param_topic_pointcloud_out;

      std::vector<KalmanFilter> objects_kf;
      std::vector<EnsembleKalmanFilter> objects_enkf;
      std::vector<EnsembleKalmanFilter> objects_;
      
      int n_, m_, c_, N_, predict_num_;
      double max_dist_;
      int publish_rate_;
      std::vector<Eigen::VectorXd> meas;
      int id_;
      std::string tracker_type_;
      std::string map_frame_;

      visualization_msgs::msg::MarkerArray centroid_array;
      visualization_msgs::msg::Marker centroid_marker;
      // rviz_visual_tools::RvizVisualTools rviz_interface;

  };
  
} // namespace tracker

#endif //TRACKER_NODE__TRACKER_NODE_NODE_HPP_