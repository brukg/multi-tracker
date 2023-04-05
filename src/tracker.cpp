// #include <tracker/kf.hpp>
#include <tracker/tracker.hpp>

namespace tracker
{

  using std::placeholders::_1;
    
  TrackerNode::TrackerNode(const rclcpp::NodeOptions& options) : Node("itav_agv_tracker_node",options) 
  {
    declare_parameter<std::string>("map_frame",map_frame_);
    declare_parameter<int>("publish_rate", 10);
    declare_parameter<std::string>("points2_1","in_1");
    declare_parameter<std::string>("points2_2","in_2");
    declare_parameter<std::string>("points2_out", "out");

    declare_parameter<double>("clustering_tolerance",0.2);
    declare_parameter<double>("min_cluster_size",5);
    declare_parameter<double>("max_cluster_size",5000);
    declare_parameter<int>("points_threshold", 10);
    declare_parameter<std::string>("cluster_search_method","kdtree");
    declare_parameter<double>("bandwidth", 0.1);
    declare_parameter<double>("convergence_threshold", 0.001);
    declare_parameter<int>("max_iterations", 100);
    declare_parameter<std::string>("tracker_type","kf");
    declare_parameter<int>("state_size", 4);
    declare_parameter<int>("control_size", 0);
    declare_parameter<int>("measurement_size", 2);
    declare_parameter<int>("ensemble_size", 10);
    declare_parameter<int>("predict_num_with_no_update", 10);
    declare_parameter<double>("max_distance", 3);
    
    map_frame_ = get_parameter("map_frame").as_string();
    publish_rate_ = get_parameter("publish_rate").as_int();
    points2_1_sub_ = get_parameter("points2_1").as_string();
    points2_2_sub_ = get_parameter("points2_2").as_string();
    param_topic_pointcloud_out = get_parameter("points2_out").as_string();

    clustering_tolerance_ = get_parameter("clustering_tolerance").as_double();
    min_cluster_size_ = get_parameter("min_cluster_size").as_double();
    max_cluster_size_ = get_parameter("max_cluster_size").as_double();
    points_threshold_ = get_parameter("points_threshold").as_int();
    cluster_search_method_ = get_parameter("cluster_search_method").as_string();
    bandwidth_ = get_parameter("bandwidth").as_double();
    convergence_threshold_ = get_parameter("convergence_threshold").as_double();
    max_iterations_ = get_parameter("max_iterations").as_int();
    tracker_type_ = get_parameter("tracker_type").as_string();
    n_ = get_parameter("state_size").as_int();
    c_ = get_parameter("control_size").as_int();
    m_ = get_parameter("measurement_size").as_int();
    N_ = get_parameter("ensemble_size").as_int();
    predict_num_ = get_parameter("predict_num_with_no_update").as_int();
    max_dist_ = get_parameter("max_distance").as_double();

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1000), rmw_qos_profile_sensor_data);
    
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
    points2_1_sub_, sensor_qos, std::bind(&tracker::TrackerNode::topicCallback, this, _1));
    first_scan_ = true; dt_ = 0.0; t_old_ = 0.0; t_new_ = 0.0; id_ = 0;
    is_update_ = false;
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrackerNode::trackObjects, this));

    RCLCPP_INFO(this->get_logger(), "\n"
                                    "Node:       tracker\n"
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
    "predict_num_with_no_update: %d\n"
    "max_distance: %f\n",
    n_, c_, m_, N_, predict_num_, max_dist_);
  }

  // void TrackerNode::trackObjects(std::vector<Eigen::VectorXd> &meas)
  void TrackerNode::trackObjects()
  { 
    // RCLCPP_INFO(this->get_logger(), "tracker node");
    RCLCPP_INFO(this->get_logger(), "trackObjects: objects_.size(): %d", objects_.size());
    if (first_scan_ && is_update_ && meas.size() != 0) {
      // RCLCPP_INFO(this->get_logger(), "initializing");
      
      for (int i = 0; i < meas.size(); i++) {
        if (tracker_type_ == "kf"){
          // objects_ = &objects_kf;
          // objects_.push_back(KalmanFilter(n_, c_, m_));

        } else if (tracker_type_ == "ekf"){
          // objects_ = objects_ekf;
          // objects_.push_back(EKF(n_, c_, m_));
        } else if (tracker_type_ == "enkf"){
          // objects_ = &objects_enkf;
          objects_.push_back(EnsembleKalmanFilter(n_, c_, m_, N_));
        }
        objects_[i].init(meas[i]);
        objects_[i].setIdentifier(id_);
        // RCLCPP_INFO(this->get_logger(), "initializing object %i id_ %i", i, objects_[i].getIdentifier());
        id_++;
      }
      first_scan_ = false;
      is_update_ = false;
    }

    std::vector<Eigen::VectorXd> pred;

    if (!first_scan_){ // if not first scan then predict
      // RCLCPP_INFO(this->get_logger(), "predicting");
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
      // RCLCPP_INFO(this->get_logger(), "updating");
      is_update_ = false;
      associateVectors(pred, meas);
      // RCLCPP_INFO(this->get_logger(), "after associateVectors: objects_.size(): %d", objects_.size());
    }

    // publish the state of the objects
    publishMarkers();
    
  }

  void TrackerNode::associateVectors(std::vector<Eigen::VectorXd>& pred, std::vector<Eigen::VectorXd>& meas)
  {
    // int num_predicted = pred.size();
    // int num_measured = meas.size();
    // Eigen::MatrixXf cost_matrix(num_predicted, num_measured);
    // RCLCPP_INFO(this->get_logger(), "associateVectors: pred.size(): %d, meas.size(): %d", pred.size(), meas.size());
    // // RCLCPP Info all the pred members in structured style
    // for (int i = 0; i < pred.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "pred[%d]: x: %f, y: %f ", i, pred[i](0), pred[i](1));
    // }
    // // RCLCPP Info all the meas members in structured style
    // for (int i = 0; i < meas.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "meas[%d]: x: %f, y: %f ", i, meas[i](0), meas[i](1));
    // }
    // RCLCPP_INFO(this->get_logger(), "max dist: %f", max_dist_);
    // // Compute cost matrix (negative distance) between pred and meas vectors
    // dlib::matrix<int> dlib_cost_matrix(num_predicted, num_measured);

    // for (int i = 0; i < num_predicted; i++) {
    //     for (int j = 0; j < num_measured; j++) {
    //         float dist = (pred[i].head(2) - meas[j].head(2)).norm();
    //         dlib_cost_matrix(i, j) = (dist <= max_dist_) ? static_cast<int>(std::round((max_dist_ - dist)*100)) : 0;//std::numeric_limits<int>::max();
    //     }
    // }
    // RCLCPP_INFO(this->get_logger(), "dlib_cost_matrix rows: %d, cols: %d", dlib_cost_matrix.nr(), dlib_cost_matrix.nc());

    // // Solve assignment problem using the Hungarian algorithm
    // std::vector<long>  assignment_matrix = dlib::max_cost_assignment(dlib_cost_matrix);

    // RCLCPP_INFO(this->get_logger(), "assignment_matrix size: %d", assignment_matrix.size());
    
    // for (int i = 0; i < assignment_matrix.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "assignment_matrix[%d]: %d", i, assignment_matrix[i]);
    // }

    // // Update pred vectors based on optimal assignment
    // for (long i = 0; i < num_predicted; ++i) {
    //   long j = assignment_matrix[i];
    //   if (j >=0 && j < num_measured) {
    //     RCLCPP_INFO(this->get_logger(), "I %d, J %d ",i, j);
    //     RCLCPP_INFO(this->get_logger(), "objects_[i].state()(0): %f, objects_[i].state()(1): %f", objects_[i].state()(0), objects_[i].state()(1));
    //     RCLCPP_INFO(this->get_logger(), "meas[j](2): %f, meas[j](3): %f", meas[j](0), meas[j](1));
    //     double vx = (meas[j](0) - objects_[i].state()(0)) / dt_;
    //     double vy = (meas[j](1) - objects_[i].state()(1)) / dt_;

    //     RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f, dt: %f", vx, vy, dt_);

    //     // // resize the measurement vector to include the velocity components
    //     meas[j].conservativeResize(4);
    //     RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f, dt: %f", vx, vy, dt_);
    //     meas[j](2) = (hypot(vx, vy) < 0.1) or (hypot(vx, vy) > 2) ? 0 : vx;
    //     meas[j](3) = (hypot(vx, vy) < 0.1 ) or (hypot(vx, vy) > 2) ? 0 : vy;
    //     // // RCLCPP_INFO(this->get_logger(), "meas[col_indices[row_indices[i]]](2): %f, meas[col_indices[row_indices[i]]](3): %f", meas[col_indices[row_indices[i]]](2), meas[col_indices[row_indices[i]]](3));
    //     objects_[i].update(meas[j]);
    //     RCLCPP_INFO(this->get_logger(), "update(): x: %f, y: %f ", objects_[i].state()(0), objects_[i].state()(1));
    //   }
    // }
    // RCLCPP_INFO(this->get_logger(), "update-----------------");
    // // Add unassociated meas vectors to the end of the pred vector list
    // for (int j = 0; j < num_measured; j++) {
    //     bool is_associated = false;
    //     for (long i = 0; i < assignment_matrix.size(); ++i) {
    //         if (assignment_matrix[i] == j) {
    //             is_associated = true;
    //             break;
    //         }
    //     }
    //     if (!is_associated) {
    //         // pred.push_back(meas[j]);
    //         if (tracker_type_ == "kf"){
    //           // objects_ = objects_kf;
    //           // objects_.push_back(KalmanFilter(n_, c_, m_));

    //         } else if (tracker_type_ == "ekf"){
    //           // objects_ = objects_ekf;
    //           // objects_.push_back(EKF(n_, c_, m_));
    //         } else if (tracker_type_ == "enkf"){
    //           // objects_ = objects_enkf;
    //           objects_.push_back(EnsembleKalmanFilter(n_, c_, m_, N_));
    //         }
            
    //         objects_.back().init(meas[j]);
    //         objects_.back().setIdentifier(id_);
    //         id_++;
    //     }
    // }

    // RCLCPP_INFO(this->get_logger(), "push_back-----------------");



    // get distances between predictions and measurements
    Eigen::MatrixXd dist(pred.size(), meas.size());
    // RCLCPP_INFO(this->get_logger(), "meas size: %d", meas.size());

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
        double vx = (meas[min_dist_idx[i]](0) - objects_[i].state()(0)) / 0.2;
        double vy = (meas[min_dist_idx[i]](1) - objects_[i].state()(1)) / 0.2;
        // RCLCPP_INFO(this->get_logger(), "vx: %f, vy: %f, dt: %f", vx, vy, dt_);
        // resize the measurement vector to include the velocity
        meas[min_dist_idx[i]].conservativeResize(4);
        meas[min_dist_idx[i]](2) = (hypot(vx, vy) < 0.1) or (hypot(vx, vy) > 2) ? 0 : vx;
        meas[min_dist_idx[i]](3) = (hypot(vx, vy) < 0.1 ) or (hypot(vx, vy) > 2) ? 0 : vy;
        objects_[i].update(meas[min_dist_idx[i]]);
      }else { // if no measurement is associated with the object, remove it
        // objects_.erase(objects_.begin() + i);
      }
      // RCLCPP_INFO(this->get_logger(), "index: %d, x: %f, y: %f vx: %f, vy: %f",
                // min_dist_idx[i], objects_[i].state()(0), objects_[i].state()(1), objects_[i].state()(2), objects_[i].state()(3));

    }

    // once the state of the object is updated,  use unassociated measurements to create new objects
    for (int i = 0; i < meas.size(); i++) {
      if (std::find(meas_idx.begin(), meas_idx.end(), i) == meas_idx.end()) {
        if (tracker_type_ == "kf"){
              // objects_ = objects_kf;
              // objects_.push_back(KalmanFilter(n_, c_, m_));

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
      // RCLCPP_INFO(this->get_logger(), "meas: %f, %f", meas[i](0), meas[i](1));
    }

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

      marker.id = i;//objects_[i].getIdentifier();
      marker.pose.position.x = objects_[i].state()(0);
      marker.pose.position.y = objects_[i].state()(1);

      marker_array.markers.push_back(marker);

      // obstacle states
      state_array.data.push_back(objects_[i].state()(0));
      state_array.data.push_back(objects_[i].state()(1));
      state_array.data.push_back(objects_[i].state()(2));
      state_array.data.push_back(objects_[i].state()(3));

      // velocity
      velocity.id = i;//objects_[i].getIdentifier();
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
    // RCLCPP_INFO(this->get_logger(), "marker published");

  }

  // ///////////////
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 
    TrackerNode::meanShiftCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  { 

    
    // Perform Mean Shift clustering

    int iter = 0;
    float max_displacement;
    do {
      max_displacement = 0.0f;
      for (size_t i = 0; i < cloud->size(); ++i) {
        pcl::PointXYZ previous_point = cloud->points[i];
        pcl::PointXYZ mean_shift = kernel_mean_shift(previous_point, cloud);
        cloud->points[i] = mean_shift;

        float displacement = euclidean_distance(previous_point, mean_shift);
        if (displacement > max_displacement) {
          max_displacement = displacement;
        }
      }
      ++iter;
    } while (iter < max_iterations_ && max_displacement > convergence_threshold_);

    RCUTILS_LOG_INFO("Mean shift done");
    // Group points into clusters based on cluster_tolerance
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<bool> processed(cloud->size(), false);

    for (size_t i = 0; i < cloud->size(); ++i) {
      if (processed[i]) {
        continue;
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      cluster->points.push_back(cloud->points[i]);
      processed[i] = true;

      for (size_t j = i + 1; j < cloud->size(); ++j) {
        if (processed[j]) {
          continue;
        }

        float distance = euclidean_distance(cloud->points[i], cloud->points[j]);
        if (distance <= clustering_tolerance_) {
          cluster->points.push_back(cloud->points[j]);
          processed[j] = true;
        }
      }

      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
    }

    return clusters;
    
  }
  float TrackerNode::euclidean_distance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return std::sqrt(
      std::pow(p1.x - p2.x, 2) +
      std::pow(p1.y - p2.y, 2) +
      std::pow(p1.z - p2.z, 2));
  }

  pcl::PointXYZ TrackerNode::kernel_mean_shift(const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointXYZ mean_shift(0, 0, 0);
    float total_weight = 0;
    // RCUTILS_LOG_INFO("kernel_mean_shift");
    for (const auto& neighbor : cloud->points) {
      float distance = euclidean_distance(point, neighbor);
      float weight = std::exp(-std::pow(distance, 2) / (2 * std::pow(bandwidth_, 2)));

      mean_shift.x += weight * neighbor.x;
      mean_shift.y += weight * neighbor.y;
      mean_shift.z += weight * neighbor.z;
      total_weight += weight;
    }

    mean_shift.x /= total_weight;
    mean_shift.y /= total_weight;
    mean_shift.z /= total_weight;
    // RCLCPP_INFO(this->get_logger(), "mean_shift.x %f", mean_shift.x);
    return mean_shift;
  }
  std::vector<Eigen::Vector2f> compute_xy_centroids(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
    std::vector<Eigen::Vector2f> centroids;
    
    for (const auto& cluster : clusters) {
      Eigen::Vector2f centroid(0, 0);
      for (const auto& point : cluster->points) {
        centroid.x() += point.x;
        centroid.y() += point.y;
      }

      centroid.x() /= cluster->points.size();
      centroid.y() /= cluster->points.size();

      centroids.push_back(centroid);
    }

  return centroids;
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

    pcl::fromROSMsg(*msg,*input);    

    // RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", input->size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    // RCLCPP_INFO(this->get_logger(), "The number of clusters is %i", clusters.size());

    std::vector<pcl::PointXYZ> centroids_marker;
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

                        
    // Insert your pcl object here
    // -----------------------------------
    if (cluster_search_method_ == "kdtree"){

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

      pcl::PointCloud<pcl::PointXYZ>::Ptr centroids (new pcl::PointCloud<pcl::PointXYZ>);
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
        centroid_marker.id = it - cluster_indices.begin();
        centroid_marker.pose.position.x = centroid.x;
        centroid_marker.pose.position.y = centroid.y;
        centroid_array.markers.push_back(centroid_marker);


        // centroids.push_back(centroid); //cluster centroid
        // 
        if (num_pts < points_threshold_) {
          centroids->points.push_back(centroid);
          clusters.push_back(cloud_cluster); //clusters
        //   double x = centroid.x;
        //   double y = centroid.y;
        //   meas.push_back(Eigen::Vector4d(x, y, 0, 0));

        }
        
        // visualise clusters in rviz
        sensor_msgs::msg::PointCloud2 cloud_cluster_msg;
        pcl::toROSMsg(*cloud_cluster,cloud_cluster_msg);
        cloud_cluster_msg.header.frame_id = map_frame_;
        cloud_cluster_msg.header.stamp = msg->header.stamp;

        cluster_pub_->publish(cloud_cluster_msg);
        
      }


      RCLCPP_INFO(this->get_logger(), "The number of clusters is %i", centroids->size());
      meas.clear();
      
      if (centroids -> size()>10){
        kdtree->setInputCloud (centroids);
        // cluster extraction
        std::vector<pcl::PointIndices> centroid_indices;
        // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (1.0);
        ec.setMinClusterSize (1);
        ec.setMaxClusterSize (20);
        ec.setSearchMethod (kdtree);
        ec.setInputCloud (centroids);
        ec.extract (centroid_indices);
        for (std::vector<pcl::PointIndices>::const_iterator it = centroid_indices.begin (); it != centroid_indices.end (); ++it) {
          float x = 0.0;
          float y = 0.0;
          int num_pts = 0;
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
            cloud_cluster->points.push_back (centroids->points[*pit]); //*
            x += centroids->points[*pit].x;
            y += centroids->points[*pit].y;
            num_pts++;
          }
          pcl::PointXYZ centroid;
          centroid.x = x/num_pts;
          centroid.y = y/num_pts;
          centroid.z = 0.0;
          meas.push_back(Eigen::Vector4d(centroid.x, centroid.y, 0, 0));


        }
      }





      
    } else if (cluster_search_method_ == "meanshift") {
      // using meanshift to cluster pointcloud
      clusters = meanShiftCluster(input);
        
      meas.clear();
      for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        pcl::PointXYZ centroid(0, 0, 0);
        int num_pts = 0;
        for (const auto& point : cluster->points) {
          centroid.x += point.x;
          centroid.y += point.y;
          num_pts++;
        // centroid.z += point.z;
        }

        centroid.x /= cluster->points.size();
        centroid.y /= cluster->points.size();

        // centroids.push_back(centroid);
        if (num_pts < points_threshold_) {
          double x = centroid.x;
          double y = centroid.y;
          meas.push_back(Eigen::Vector4d(x, y, 0, 0));
          centroid_marker.id = i;
          centroid_marker.pose.position.x = centroid.x;
          centroid_marker.pose.position.y = centroid.y;
          centroid_array.markers.push_back(centroid_marker);
        }

        // visualise clusters in rviz
        sensor_msgs::msg::PointCloud2 cloud_cluster_msg;
        pcl::toROSMsg(*cluster,cloud_cluster_msg);
        cloud_cluster_msg.header.frame_id = msg->header.frame_id;
        cloud_cluster_msg.header.stamp = msg->header.stamp;

        cluster_pub_->publish(cloud_cluster_msg);
      }
    }
    



    // meas = compute_xy_centroids(clusters);
    // std::vector<pcl::PointXYZ> centroids;

    // RCLCPP_INFO(this->get_logger(), "no of clusters: %li", clusters.size());
    // RCLCPP_INFO(this->get_logger(), "no of meas: %li", meas.size());
    centroid_pub_->publish(centroid_array);


    
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
  rclcpp::spin(std::make_shared<tracker::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
