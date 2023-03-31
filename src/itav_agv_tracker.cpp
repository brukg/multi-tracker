// #include <itav_agv_tracker/kf.hpp>
#include <itav_agv_tracker/itav_agv_tracker.hpp>

namespace itav_agv_tracker
{

  using std::placeholders::_1;
    
  TrackerNode::TrackerNode(const rclcpp::NodeOptions& options) : Node("itav_agv_tracker_node",options) 
  {
    declare_parameter<std::string>("map_frame",map_frame_);
    declare_parameter<std::string>("points2_1","in_1");
    declare_parameter<std::string>("points2_2","in_2");
    declare_parameter<std::string>("points2_out", "out");

    declare_parameter<double>("clustering_tolerance",0.2);
    declare_parameter<double>("min_cluster_size",5);
    declare_parameter<double>("max_cluster_size",5000);
    declare_parameter<int>("points_threshold", 10);
    declare_parameter<std::string>("cluster_search_method","kdtree");
    
    declare_parameter<std::string>("tracker_type","kf");
    declare_parameter<int>("state_size", 4);
    declare_parameter<int>("control_size", 0);
    declare_parameter<int>("measurement_size", 2);
    declare_parameter<int>("ensemble_size", 10);
    declare_parameter<int>("predict_num_with_no_update", 10);
    declare_parameter<int>("max_dist", 3);
    
    map_frame_ = get_parameter("map_frame").as_string();
    points2_1_sub_ = get_parameter("points2_1").as_string();
    points2_2_sub_ = get_parameter("points2_2").as_string();
    param_topic_pointcloud_out = get_parameter("points2_out").as_string();

    clustering_tolerance_ = get_parameter("clustering_tolerance").as_double();
    min_cluster_size_ = get_parameter("min_cluster_size").as_double();
    max_cluster_size_ = get_parameter("max_cluster_size").as_double();
    points_threshold_ = get_parameter("points_threshold").as_int();
    cluster_search_method_ = get_parameter("cluster_search_method").as_string();
    tracker_type_ = get_parameter("tracker_type").as_string();
    n_ = get_parameter("state_size").as_int();
    c_ = get_parameter("control_size").as_int();
    m_ = get_parameter("measurement_size").as_int();
    N_ = get_parameter("ensemble_size").as_int();
    predict_num_ = get_parameter("predict_num_with_no_update").as_int();
    max_dist_ = get_parameter("max_dist").as_int();

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
    
    // publishers
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,1);
    objects_id_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("objects_id",1);
    state_size_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("state_size",1);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracked_objects",1);
    centroid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("centroid",1);
    objects_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("obstacles_state",1);
    velocity_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles_velocity_marker",1);
    // rviz_interface(new rviz_visual_tools::RvizVisualTools("map","/tracked_objects", this, const RemoteControlPtr& remote_control = RemoteControlPtr()));
    
    // subscribers
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    points2_1_sub_, sensor_qos, std::bind(&itav_agv_tracker::TrackerNode::topicCallback, this, _1));
    first_scan_ = true; dt_ = 0.0; t_old_ = 0.0; t_new_ = 0.0; id_ = 0;
    is_update_ = false;
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrackerNode::trackObjects, this), timer_cb_group_);

    RCLCPP_INFO(this->get_logger(), "\n"
                                    "Node:       itav_agv_tracker\n"
                                    "Subscribes: Pointcloud2 message: %s\n"
                                    "Publishes:  Pointcloud2 message: %s \n"
                                    "Running...",
                      points2_1_sub_.c_str(), param_topic_pointcloud_out.c_str());

    RCLCPP_INFO(this->get_logger(), "clustering parameters: \n"
    "clustering_tolerance: %f\n"
    "min_cluster_size: %f\n"
    "max_cluster_size: %f\n"
    "points_threshold: %d\n"
    "cluster_search_method: %s\n",
    clustering_tolerance_, min_cluster_size_, max_cluster_size_, points_threshold_, cluster_search_method_.c_str());

    RCLCPP_INFO(this->get_logger(), "kf parameters: \n"
    "state_size: %d\n"
    "control_size: %d\n"
    "measurement_size: %d\n"
    "ensemble_size: %d\n"
    "predict_num_with_no_update: %d\n",
    n_, c_, m_, N_, predict_num_);
  }

  // void TrackerNode::trackObjects(std::vector<Eigen::VectorXd> &meas)
  void TrackerNode::trackObjects()
  { 
    RCLCPP_INFO(this->get_logger(), "trackObjects: objects_.size(): %d", objects_.size());
    if (first_scan_) {
      
      for (int i = 0; i < meas.size(); i++) {
        // objects_.push_back(KalmanFilter(n_, c_, m_));
        objects_.push_back(EnsembleKalmanFilter(n_, c_, m_, N_));
        objects_[i].init(meas[i]);
        objects_[i].setIdentifier(id_);
        RCLCPP_INFO(this->get_logger(), "initializing object %i id_ %i", i, objects_[i].getIdentifier());
        id_++;
      }
      first_scan_ = false;
      is_update_ = false;
    }

    std::vector<Eigen::VectorXd> pred;

    if (!first_scan_){ // if not first scan then predict

      for (int i =0; i < objects_.size(); i++) {
        // RCLCPP_INFO(this->get_logger(), "predict(): x: %f, y: %f ", objects_[i].state()(0), objects_[i].state()(1));
        if (objects_[i].getCounter() < predict_num_) {
          objects_[i].setDt(dt_);
          objects_[i].predict();
          pred.push_back(objects_[i].state());
        } else { //remove object
          objects_.erase(objects_.begin() + i);
        }
      }

    }


    if (is_update_ && meas.size() != 0) {
      is_update_ = false;
      // get distances between predictions and measurements
      Eigen::MatrixXd dist(pred.size(), meas.size());
      RCLCPP_INFO(this->get_logger(), "meas size: %d", meas.size());

      // calculate distances between predictions and measurements and store in dist
      for (int i = 0; i < pred.size(); i++) {
        for (int j = 0; j < meas.size(); j++)
        {
          double dx = pred[i](0) - meas[j](0);
          double dy = pred[i](1) - meas[j](1);
          dist(i, j) = hypot(dx, dy);
        }
      }

      // find indices of minimum distance
      std::vector<int> min_dist_idx;
      std::vector<int> min_dist_vec;
      std::vector<int> meas_idx;
      for (int i = 0; i < pred.size(); i++) {
        int min_idx = -1;
        double min_dist =  max_dist_;
        for (int j = 0; j < meas.size(); j++) {
          if (dist(i, j) < min_dist) {
            if (std::find(meas_idx.begin(), meas_idx.end(), j) == meas_idx.end())
            {           
              min_dist = dist(i, j);
              min_idx = j;
              meas_idx.push_back(j);
            } else {
              // // check if the previous measurement is closer
              // int prev_idx = std::find(meas_idx.begin(), meas_idx.end(), j) - meas_idx.begin();
              // if (dist(i, j) < min_dist_vec[prev_idx]) {
              //   min_dist = dist(i, j);
              //   min_idx = j;
              //   meas_idx.push_back(j);
              //   min_dist_idx[prev_idx] = -1;
              // } else { // if the previous measurement is closer, ignore this measurement
              //   min_idx = -1;
              // }
            }
          } 
        }
        min_dist_idx.push_back(min_idx);
        min_dist_vec.push_back(min_dist);
      }

      // update the state of the object with the measurement
      for (int i = 0; i < pred.size(); i++) {
        if (min_dist_idx[i] != -1) {
          // estimate the velocity
          Eigen::Vector2d vel;
          double vx = (meas[min_dist_idx[i]](0) - objects_[i].state()(0)) / dt_;
          double vy = (meas[min_dist_idx[i]](1) - objects_[i].state()(1)) / dt_;
          vel << vx, vy;
          RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f, dt: %f", vx, vy, dt_);
          // resize the measurement vector to include the velocity
          meas[min_dist_idx[i]].conservativeResize(4);
          meas[min_dist_idx[i]](2) = (hypot(vx, vy) < 0.1) or (hypot(vx, vy) > 2) ? 0 : vx;
          meas[min_dist_idx[i]](3) = (hypot(vx, vy) < 0.1 ) or (hypot(vx, vy) > 2) ? 0 : vy;
          objects_[i].update(meas[min_dist_idx[i]]);
        }else { // if no measurement is associated with the object, remove it
          // objects_.erase(objects_.begin() + i);
        }
        RCLCPP_INFO(this->get_logger(), "index: %d, x: %f, y: %f vx: %f, vy: %f",
                  min_dist_idx[i], objects_[i].state()(0), objects_[i].state()(1), objects_[i].state()(2), objects_[i].state()(3));
 
      }

      // once the state of the object is updated,  use unassociated measurements to create new objects
      for (int i = 0; i < meas.size(); i++) {
        if (std::find(meas_idx.begin(), meas_idx.end(), i) == meas_idx.end()) {
          if (tracker_type_ == "kf"){
            objects_ = objects_kf;
            objects_.push_back(KalmanFilter(n_, c_, m_));

          } else if (tracker_type_ == "ekf"){
            // objects_ = objects_ekf;
            // objects_.push_back(EKF(n_, c_, m_));
          } else if (tracker_type_ == "enkf"){
            // objects_ = objects_enkf;
            objects_.push_back(EnsembleKalmanFilter(n_, c_, m_, N_));
          }
          
          objects_.back().init(meas[i]);
          objects_.back().setIdentifier(id_);
          id_++;
        }
        RCLCPP_INFO(this->get_logger(), "meas: %f, %f", meas[i](0), meas[i](1));
      }

      RCLCPP_INFO(this->get_logger(), "update:");
    }

    // publish the state of the objects
    publishMarkers();
    
  }
  
  void TrackerNode::publishMarkers()
  {

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->now();
    marker.ns = "objects";
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.clear();

    std_msgs::msg::Float32MultiArray state_array;
    state_array.data.clear();
    
    visualization_msgs::msg::MarkerArray velocity_array;
    visualization_msgs::msg::Marker velocity;
    velocity.header.frame_id = map_frame_;
    velocity.header.stamp = this->now();
    velocity.ns = "velocity";
    velocity.type = visualization_msgs::msg::Marker::ARROW;
    velocity.action = visualization_msgs::msg::Marker::DELETEALL;
    velocity.scale.x = 0.1;
    velocity.scale.y = 0.1;
    velocity.scale.z = 0.05;
    velocity.color.a = 1.0;
    velocity.color.r = 0.0;
    velocity.color.g = 1.0;
    velocity.color.b = 0.0;
    velocity_array.markers.clear();

    marker_pub_->publish(marker_array); // marker publish
    marker_pub_->publish(velocity_array); // velocity publish
    // rviz_interface.deleteAllMarkers();
    marker.action = visualization_msgs::msg::Marker::ADD;
    velocity.action = visualization_msgs::msg::Marker::ADD;
    // objects id push
    std_msgs::msg::Int32MultiArray objects_id_;
    objects_id_.data.clear();
    
    for (int i = 0; i < objects_.size(); i++) {

      marker.id = objects_[i].getIdentifier();
      marker.pose.position.x = objects_[i].state()(0);
      marker.pose.position.y = objects_[i].state()(1);

      marker_array.markers.push_back(marker);

      // obstacle states
      state_array.data.push_back(objects_[i].state()(0));
      state_array.data.push_back(objects_[i].state()(1));
      state_array.data.push_back(objects_[i].state()(2));
      state_array.data.push_back(objects_[i].state()(3));

      // velocity
      velocity.id = objects_[i].getIdentifier();
      velocity.pose.position.x = objects_[i].state()(0);
      velocity.pose.position.y = objects_[i].state()(1);
      velocity.pose.position.z = 0.1;
      double theta = atan2(objects_[i].state()(3), objects_[i].state()(2));
      tf2::Quaternion qt;
      qt.setRPY(0, 0, theta);
      velocity.pose.orientation.x = qt.x();
      velocity.pose.orientation.y = qt.y();
      velocity.pose.orientation.z = qt.z();
      velocity.pose.orientation.w = qt.w();
      velocity.scale.x = hypot(objects_[i].state()(2), objects_[i].state()(3)) +0.0001;
      velocity_array.markers.push_back(velocity);

      // objects id push
      objects_id_.data.push_back(objects_[i].getIdentifier());
      

    }
    marker_pub_->publish(marker_array); // marker publish
    objects_state_pub_->publish(state_array); // state publish
    marker_pub_->publish(velocity_array); // velocity publish
    objects_id_pub_->publish(objects_id_); // objects id publish

    std_msgs::msg::Int32MultiArray state_size;
    state_size.data.clear();
    state_size.data.push_back(objects_.size());
    state_size_pub_->publish(state_size);
    RCLCPP_INFO(this->get_logger(), "marker published");

  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 
    TrackerNode::clusterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    
  }

  void TrackerNode::topicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // unsigned int num_points = msg->width;
    // RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
      

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl kdtree object
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    // ROS2 Pointcloud2 to PCL Pointcloud2
    
    pcl::fromROSMsg(*msg,*input);    
                        
    // Insert your pcl object here
    // -----------------------------------
    kdtree->setInputCloud (input);
    // cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clustering_tolerance_);
    ec.setMinClusterSize (min_cluster_size_);
    ec.setMaxClusterSize (max_cluster_size_);
    ec.setSearchMethod (kdtree);
    ec.setInputCloud (input);
    ec.extract (cluster_indices);
    
    // using dbscan to cluster pointcloud




    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<pcl::PointXYZ> centroids;
    centroid_marker.header.frame_id = map_frame_;
    centroid_marker.header.stamp = this->now();
    centroid_marker.ns = "centroids";
    centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
    centroid_marker.action = visualization_msgs::msg::Marker::ADD;
    centroid_marker.scale.x = 0.2;
    centroid_marker.scale.y = 0.2;
    centroid_marker.scale.z = 0.3;
    centroid_marker.color.a = 1.0;
    centroid_marker.color.r = 1.0;
    centroid_marker.color.g = 0.0;
    centroid_marker.color.b = 0.0;
    centroid_array.markers.clear();
    meas.clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      float x = 0.0;
      float y = 0.0;
      int num_pts = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
        cloud_cluster->points.push_back (input->points[*pit]); //*
        x += input->points[*pit].x;
        y += input->points[*pit].y;
        num_pts++;
      }
      pcl::PointXYZ centroid;
      centroid.x = x/num_pts;
      centroid.y = y/num_pts;
      centroid.z = 0.0;
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusters.push_back(cloud_cluster); //clusters
      centroids.push_back(centroid); //cluster centroid
      // 
      if (num_pts < points_threshold_) {
        meas.push_back(Eigen::Vector2d(centroid.x, centroid.y));
        centroid_marker.id = it - cluster_indices.begin();
        centroid_marker.pose.position.x = centroid.x;
        centroid_marker.pose.position.y = centroid.y;
        centroid_array.markers.push_back(centroid_marker);
      }
      
      // visualise clusters in rviz
      sensor_msgs::msg::PointCloud2 cloud_cluster_msg;
      pcl::toROSMsg(*cloud_cluster,cloud_cluster_msg);
      cloud_cluster_msg.header.frame_id = msg->header.frame_id;
      cloud_cluster_msg.header.stamp = msg->header.stamp;

      cluster_pub_->publish(cloud_cluster_msg);
      
    }

    RCLCPP_INFO(this->get_logger(), "no of clusters: %li", clusters.size());
    RCLCPP_INFO(this->get_logger(), "no of meas: %li", meas.size());
    centroid_pub_->publish(centroid_array);
    // if meas are close to each other, then they are the same object
    for (int i = 0; i < meas.size(); i++) {
      for (int j = i+1; j < meas.size(); j++) {
        if (meas[i].isApprox(meas[j], 0.1)) {
          // merge the two measurements
          meas[i] = (meas[i] + meas[j])/2;
          // remove the second measurement
          meas.erase(meas.begin() + j);
          j--;
        }
      }
    }
    
    if (first_scan_)
        t_old_ = msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9; // first time
    else  {
      t_new_ = msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9; // current time
      dt_ = t_new_ - t_old_; // time difference
      t_old_ = t_new_; // update time
    }
    is_update_ = true;
  
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<itav_agv_tracker::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
