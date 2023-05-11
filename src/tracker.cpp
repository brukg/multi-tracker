#include <tracker/tracker.hpp>

namespace tracker
{

  // using namespace std::placeholders;
    
  TrackerNode::TrackerNode(const rclcpp::NodeOptions& options) 
  : Node("itav_agv_tracker_node",options)
  { 
    declare_parameter<std::string>("map_frame","map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<int>("publish_rate", 10);
    declare_parameter<bool>("visualize", true);
    declare_parameter<std::string>("points2_1","in_1");
    declare_parameter<std::string>("points2_2","in_2");
    declare_parameter<std::string>("points2_out", "out");

    declare_parameter<double>("scanner_range", 10.0);
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
    declare_parameter<std::string>("data_association_type","jdpa");
    declare_parameter<int>("predict_num_with_no_update", 10);
    declare_parameter<double>("max_distance", 3);
    declare_parameter<double>("max_expected_velocity", 1);
    declare_parameter<int>("sliding_window_size", 10);

    visualise_ = get_parameter("visualize").as_bool();
    map_frame_ = get_parameter("map_frame").as_string();
    robot_base_frame_ = get_parameter("base_frame").as_string();
    publish_rate_ = get_parameter("publish_rate").as_int();
    points2_1_sub_ = get_parameter("points2_1").as_string();
    points2_2_sub_ = get_parameter("points2_2").as_string();
    param_topic_pointcloud_out = get_parameter("points2_out").as_string();

    range_ = get_parameter("scanner_range").as_double();
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
    sliding_window_size_ = get_parameter("sliding_window_size").as_int();
    association_type_ = get_parameter("data_association_type").as_string();
    predict_num_ = get_parameter("predict_num_with_no_update").as_int();
    max_dist_ = get_parameter("max_distance").as_double();
    max_expected_velocity_ = get_parameter("max_expected_velocity").as_double();

    
    
    // publishers
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out,1);
    objects_id_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("tracker/objects_id",1);
    state_size_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("tracker/state_size",1);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracker/tracked_objects",1);
    centroid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tracker/centroid",1);
    objects_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("tracker/obstacles_state",1);
    // point_particles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("point_particles",1);
    
    // subscribers
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(2), rmw_qos_profile_sensor_data);
    scan_sub_1_.subscribe(this, points2_1_sub_, rmw_qos_profile_sensor_data);
    scan_sub_2_.subscribe(this, points2_2_sub_, rmw_qos_profile_sensor_data);
    RCLCPP_INFO(this->get_logger(), "Subscribing to %s and %s", points2_1_sub_.c_str(), points2_2_sub_.c_str());
    sync_.reset(new message_filters::Synchronizer<approximate_policy>(approximate_policy(1), scan_sub_1_, scan_sub_2_));
    sync_->registerCallback(std::bind(&TrackerNode::scanCallback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s and %s", points2_1_sub_.c_str(), points2_2_sub_.c_str());
    // subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //                         points2_1_sub_, sensor_qos, std::bind(&tracker::TrackerNode::scanCallback, this, std::placeholders::_1));
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                            "point_cloud", sensor_qos, std::bind(&tracker::TrackerNode::pointCloudCallback, this, std::placeholders::_1));
    
    first_scan_ = true; dt_ = 0.0; t_old_ = 0.0; t_new_ = 0.0; id_ = 0;
    is_update_ = false;
    
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(int(1000/publish_rate_)), 
                                    std::bind(&TrackerNode::trackObjects, this));

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    old_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    temp = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    RCLCPP_INFO(this->get_logger(), "\n"
                                    "Node:       tracker\n"
                                    "Subscribes: Pointcloud2 message: %s\n"
                                    "Publishes:  Pointcloud2 message: %s \n"
                                    "visualise:  %s\n"
                                    "Running...",
                      points2_1_sub_.c_str(), param_topic_pointcloud_out.c_str(), visualise_ ? "true" : "false");

    
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
        objects_[i].setRadius(hypot(meas[i](2), meas[i](3)) / 2);
        objects_[i].setWidthHeight(meas[i](2), meas[i](3));
        objects_[i].setSlidingWindowSize(sliding_window_size_);
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
        // RCLCPP_INFO(this->get_logger(), "predict(): x: %f, y: %f ", objects_[i].getState()(0), objects_[i].getState()(1));
        if (objects_[i].getCounter() < predict_num_) {
          objects_[i].setDt(1/publish_rate_);
          objects_[i].predict();
          pred.push_back(objects_[i].getState());
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
      // publish the state of the objects
    }
    if (visualise_) publishMarkers();

    
  }
  
  // is there a bug in the associateVectors function?
  void TrackerNode::associateVectors(std::vector<Eigen::VectorXd>& pred, std::vector<Eigen::VectorXd>& meas)
  {
    // Use the JPDA-based function to obtain the associations between predictions and measurements
    double gate_threshold = max_dist_;
    std::vector<int> associations;
    if (association_type_ == "jpda") {
      associations = JPDAAssociate(pred, meas, gate_threshold);
    } else if (association_type_ == "greedy") {
      associations = GreedyAssociate(pred, meas, gate_threshold);
    } else if (association_type_ == "hungarian") {
      associations = HungarianAssociate(pred, meas, gate_threshold);
    } else if (association_type_ == "jcbb") {
      associations = JCBBAssociate(pred, meas, gate_threshold);
    } 
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown association type: %s", association_type_.c_str());
    }

    // update the state of the object with the measurement
    #pragma omp parallel for
    for (int i = 0; i < pred.size(); i++) {
      int associated_measurement = associations[i];
        if (associated_measurement != -1 && associated_measurement < meas.size() && associated_measurement >= 0) {
          // RCLCPP_INFO(this->get_logger(), "i: %d, associated_measurement: %d", i, associated_measurement);
          // RCLCPP_INFO(this->get_logger(), "pred: %f, %f, meas: %f, %f", objects_[i].getState()(0), objects_[i].getState()(1), meas[associated_measurement](0), meas[associated_measurement](1));

          double dx = (meas[associated_measurement](0) - objects_[i].getPrevState()(0));
          double dy = (meas[associated_measurement](1) - objects_[i].getPrevState()(1));
          double d = hypot(dx, dy);
          // RCLCPP_INFO(this->get_logger(), "d: %f", d);
          double vx = dx / dt_;
          double vy = dy / dt_;
          double v = hypot(vx, vy);
          // RCLCPP_INFO(this->get_logger(), "v: %f", v);
          // previous velocity for first order filter
          double v_x_prv = objects_[i].getPrevState()(2);
          double v_y_prv = objects_[i].getPrevState()(3);
          double v_prv = hypot(v_x_prv, v_y_prv);

          // RCLCPP_INFO(this->get_logger(), "v: %f, v_prv: %f", v, v_prv);
          // resize the measurement vector to include the velocity
          // meas[associated_measurement].conservativeResize(4);
          objects_[i].setRadius(hypot(meas[associated_measurement](2), meas[associated_measurement](3)) / 2);
          objects_[i].setWidthHeight(meas[associated_measurement](2), meas[associated_measurement](3));
          double radius =  objects_[i].getRadius();
          // RCLCPP_INFO(this->get_logger(), "radius: %f, d: %f, dt: %f", radius, d, dt_);
          // if (d> radius || d >   1.0) {
          if (d > max_dist_ || abs(v - v_prv) > max_expected_velocity_) {
            // meas[associated_measurement](0) = objects_[i].getState()(0);
            // meas[associated_measurement](1) = objects_[i].getState()(1);
            // meas[associated_measurement](2) = 0;
            // meas[associated_measurement](3) = 0;
          } else {
            objects_[i].vx_.accumulate(vx);
            objects_[i].vy_.accumulate(vy);
            // meas[associated_measurement](2) = (1-0.5)*objects_[i].getPrevState()(2) + (0.5*vx);
            // meas[associated_measurement](3) = (1-0.5)*objects_[i].getPrevState()(3) + (0.5*vy);
            meas[associated_measurement](2) = objects_[i].vx_.getRollingMean();
            meas[associated_measurement](3) = objects_[i].vy_.getRollingMean();
            objects_[i].update(meas[associated_measurement]);
          }

        }else { // if no measurement is associated with the object, remove it
          // objects_.erase(objects_.begin() + i);
        }
      // RCLCPP_INFO(this->get_logger(), "index: %d, x: %f, y: %f vx: %f, vy: %f",
                // min_dist_idx[i], objects_[i].getState()(0), objects_[i].getState()(1), objects_[i].getState()(2), objects_[i].getState()(3));

    }

    // once the state of the object is updated,  use unassociated measurements to create new objects
    #pragma omp parallel for
    for (int i = 0; i < meas.size(); i++) {
      if (std::find(associations.begin(), associations.end(), i) == associations.end()) {
        // RCLCPP_INFO(this->get_logger(), "new object");
        if (tracker_type_ == "kf"){
          // objects_ = objects_kf;
          // objects_.push_back(KalmanFilter(n_, c_, m_));

        } else if (tracker_type_ == "ekf"){
          // objects_ = objects_ekf;
          // objects_.push_back(EKF(n_, c_, m_));
        } else if (tracker_type_ == "enkf"){
          // objects_ = objects_enkf;
          objects_.push_back(EnsembleKalmanFilter(n_, c_, m_, N_)); // initialize the object
        }
        meas[i](2) = 0; // initialize velocity to zero
        meas[i](3) = 0; // initialize velocity to zero
        objects_.back().init(meas[i]); // initialize the state of the object
        objects_.back().setIdentifier(id_); // set the id of the object
        objects_.back().setWidthHeight(meas[i](2), meas[i](3)); // set the width and height of the object
        objects_.back().setSlidingWindowSize(sliding_window_size_); // set the sliding window size
        id_++;
      }
      // RCLCPP_INFO(this->get_logger(), "meas: %f, %f", meas[i](0), meas[i](1));
    }

  }

  void TrackerNode::clusterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, std_msgs::msg::Header header)
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    // RCLCPP_INFO(this->get_logger(), "The number of clusters is %i", clusters.size());
    // pcl kdtree object
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);

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
      meas.clear();

      pcl::PointCloud<pcl::PointXYZ>::Ptr centroids (new pcl::PointCloud<pcl::PointXYZ>);
      #pragma omp parallel for
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        float x = 0.0;
        float y = 0.0;
        int num_pts = 0;
        double radius = 0.0;
        double max_x_, max_y_, min_x_, min_y_;
        min_x_ = max_x_ = input->points[it->indices[0]].x;
        min_y_ = max_y_ = input->points[it->indices[0]].y;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
          cloud_cluster->points.push_back (input->points[*pit]); //*

          if (input->points[*pit].x > max_x_) max_x_ = input->points[*pit].x;
          if (input->points[*pit].x < min_x_) min_x_ = input->points[*pit].x;
          if (input->points[*pit].y > max_y_) max_y_ = input->points[*pit].y;
          if (input->points[*pit].y < min_y_) min_y_ = input->points[*pit].y;
          
          x += input->points[*pit].x;
          y += input->points[*pit].y;
          num_pts++;
        }
        pcl::PointXYZ centroid;
        double x_pt = x/num_pts, y_pt = y/num_pts;
        centroid.x = x_pt;
        centroid.y = y_pt;
        centroid.z = 0.0;
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        centroid_marker.id = it - cluster_indices.begin();
        centroid_marker.pose.position.x = x_pt;
        centroid_marker.pose.position.y = y_pt;
        centroid_array.markers.push_back(centroid_marker);

        // centroids.push_back(centroid); //cluster centroid
        // 
        if (num_pts < points_threshold_) {
          centroids->points.push_back(centroid);
          clusters.push_back(cloud_cluster); //clusters

          double width = max_x_ - min_x_;
          double height = max_y_ - min_y_;
          double center_x = (max_x_ + min_x_)/2;
          double center_y = (max_y_ + min_y_)/2;
          double R_ = hypot(width, height)/2;
          // if (radius > 0.0) 
          meas.push_back(Eigen::Vector4d(center_x, center_y, (width+0.0025), (height+0.0025)));
          // meas.push_back(Eigen::Vector4d(centroid.x, centroid.y, R_, 0));

        }
        
        // visualise clusters in rviz
        sensor_msgs::msg::PointCloud2 cloud_cluster_msg;
        pcl::toROSMsg(*cloud_cluster, cloud_cluster_msg);
        cloud_cluster_msg.header.frame_id = map_frame_;
        cloud_cluster_msg.header.stamp = header.stamp;

        if (visualise_) cluster_pub_->publish(cloud_cluster_msg);
        
      }


      // RCLCPP_INFO(this->get_logger(), "The number of clusters is %i", centroids->size());
      
      
      // if (centroids -> size()>1){
      //   kdtree->setInputCloud (centroids);
      //   // cluster extraction
      //   std::vector<pcl::PointIndices> centroid_indices;
      //   // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      //   ec.setClusterTolerance (0.750);
      //   ec.setMinClusterSize (1);
      //   ec.setMaxClusterSize (20);
      //   ec.setSearchMethod (kdtree);
      //   ec.setInputCloud (centroids);
      //   ec.extract (centroid_indices);
      //   for (std::vector<pcl::PointIndices>::const_iterator it = centroid_indices.begin (); it != centroid_indices.end (); ++it) {
      //     float x = 0.0;
      //     float y = 0.0;
      //     int num_pts = 0;
      //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      //       cloud_cluster->points.push_back (centroids->points[*pit]); //*
      //       x += centroids->points[*pit].x;
      //       y += centroids->points[*pit].y;

      //       num_pts++;
      //     }
          
      //     pcl::PointXYZ centroid;
      //     centroid.x = x/num_pts;
      //     centroid.y = y/num_pts;
      //     centroid.z = 0.0;
      //     double radius = 0.0;
      //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      //       double dist = sqrt(pow(centroids->points[*pit].x - centroid.x, 2) + pow(centroids->points[*pit].y - centroid.y, 2));
      //       RCLCPP_INFO(this->get_logger(), "The distance is %f", dist);
      //       if (dist > radius){
      //         radius = dist;
      //         RCLCPP_INFO(this->get_logger(), "The radius is %f", radius);
      //       }
      //     }
      //     if (radius > 0.0) meas.push_back(Eigen::Vector4d(x, y, radius, 0));

      //     // meas.push_back(Eigen::Vector4d(centroid.x, centroid.y, radius, 0));


      //   }
      // }

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
        cloud_cluster_msg.header.frame_id = header.frame_id;
        cloud_cluster_msg.header.stamp = header.stamp;

        if (visualise_) cluster_pub_->publish(cloud_cluster_msg);
      }
    } else if (cluster_search_method_ =="dbscan") {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
      copyPointCloud(*input, *cloud_);
      dbScanSpace::dbscan dbscan;
      std::vector<htr::Point3D> groupA;
      int octreeResolution = 10;
      float eps = 3;
      int minPtsAux = 1;
      int minPts = 1;
      // convert input to cloud
      dbscan.init(groupA, cloud_, octreeResolution, eps, minPtsAux, minPts);
      // RCLCPP_INFO(this->get_logger(), "The number of points is %i", cloud_->points.size());
      // dbscan.init(groupA, cloud, cloud->points.size() * 0.001, eps, 5, 100);
      // dbscan.init(groupA, cloud, cloud->points.size() * 0.001, eps, 5, 100);
      dbscan.generateClusters();
      size_t num_clusters = dbscan.getClusters().size();
      std::vector<dbScanSpace::cluster> clts = dbscan.getClusters();
      meas.clear();
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (size_t j = 0; j < clts.size(); ++j) {
        pcl::PointXYZ point;
        point.x = clts[j].centroid.x;
        point.y = clts[j].centroid.y;
        point.z = clts[j].centroid.z;
        
        centroid_marker.id = j;
        centroid_marker.pose.position.x = point.x;
        centroid_marker.pose.position.y = point.y;
        centroid_array.markers.push_back(centroid_marker);


        meas.push_back(Eigen::Vector4d(clts[j].centroid.x, clts[j].centroid.y, 0, 0));
        cloud_cluster->points.push_back(point);
      }
      //   cloud_cluster->width = cloud_cluster->points.size();
      //   cloud_cluster->height = 1;
      //   cloud_cluster->is_dense = true;
      //   clusters.push_back(cloud_cluster);
      //   RCLCPP_INFO(this->get_logger(), "The number of points in cluster %i is %i", i, cloud_cluster->points.size());
      // }
      // RCLCPP_INFO(this->get_logger(), "The number of clusters is %i", num_clusters);
      

    }
    



    // meas = compute_xy_centroids(clusters);
    // std::vector<pcl::PointXYZ> centroids;

    // RCLCPP_INFO(this->get_logger(), "no of clusters: %li", clusters.size());
    // RCLCPP_INFO(this->get_logger(), "no of meas: %li", meas.size());
    if (visualise_)  centroid_pub_->publish(centroid_array);


    
    if (first_scan_)
        t_old_ = header.stamp.sec + header.stamp.nanosec*1e-9; // first time
    else  {
      t_new_ = header.stamp.sec + header.stamp.nanosec*1e-9; // current time
      dt_ = t_new_ - t_old_; // time difference
      t_old_ = t_new_; // update time
    }
    is_update_ = true;
    // clear the old cloud
    // copyPointCloud(*input, *old_cloud);
    // RCLCPP_INFO(this->get_logger(), "The number of points in old cloud is %i", old_cloud->points.size());

  

  }
  void TrackerNode::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg, const sensor_msgs::msg::LaserScan::ConstSharedPtr msg2)
  {
    // unsigned int num_points = msg->width;
    // RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", num_points);
    sensor_msgs::msg::PointCloud2::SharedPtr in_1(new sensor_msgs::msg::PointCloud2);
    sensor_msgs::msg::PointCloud2::SharedPtr in_2(new sensor_msgs::msg::PointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input2 (new pcl::PointCloud<pcl::PointXYZ>);

    // sensor_msgs::msg::LaserScan cropped_scan = *msg;
    // sensor_msgs::msg::LaserScan cropped_scan2 = *msg2;

    // for (int i = 0; i < msg->ranges.size(); i++) {
    //   if (msg->ranges[i] > 20.) {
    //     cropped_scan.ranges[i] = 0;
    //   }
    // }
    // for (int i = 0; i < msg2->ranges.size(); i++) {
    //   if (msg2->ranges[i] > 20.) {
    //     cropped_scan2.ranges[i] = 0;
    //   }
    // }
    // // crop laser scan by distance


    try {
      // waitForTransform

      tf_buffer_->canTransform(
        map_frame_, msg->header.frame_id,
        tf2::TimePointZero);
      // tf_buffer_->lookupTransform(
      //   map_frame_, msg->header.frame_id,
      //   tf2::TimePointZero);
      projector_1.transformLaserScanToPointCloud(map_frame_, *msg, *in_1, *tf_buffer_, range_);

      tf_buffer_->canTransform(
        map_frame_, msg2->header.frame_id,
        tf2::TimePointZero);
      // tf_buffer_->lookupTransform(
      //   map_frame_, msg2->header.frame_id,
      //   tf2::TimePointZero);
      projector_2.transformLaserScanToPointCloud(map_frame_, *msg2, *in_2, *tf_buffer_, range_);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "could not transform scan to map frame: %s", ex.what());
      return;
    }

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    pcl::fromROSMsg(*in_1,*input);
    pcl::fromROSMsg(*in_2,*input2);

    // merge the two pointclouds
    *input += *input2;


    // Look up for the transformation between target_frame and source_frame
    // try {
    //   T = tf_buffer_->lookupTransform(
    //     map_frame_, msg->header.frame_id,
    //     tf2::TimePointZero);
    // } catch (const tf2::TransformException & ex) {
    //   RCLCPP_INFO(
    //     this->get_logger(), "Could not transform %s to %s: %s",
    //     map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    //   return;
    // }
    // transform the pointcloud to map frame
    // pcl_ros::transformPointCloud(*input, *input, T);


    // Create the filtering object
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud (cloud);
    // sor.setLeafSize (0.05f, 0.05f, 0.01f);
    // sor.filter (*cloud_filtered);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    if (!first_scan_)  {
      *temp = *input;
      *input += *old_cloud;
      }

    // sor.setInputCloud (input);
    // sor.setMeanK (150);
    // sor.setStddevMulThresh (2.0);
    // sor.filter (*input);
    // RCLCPP_INFO(this->get_logger(), "The number of points in the input pointcloud is %i", input->size());
    std_msgs::msg::Header header;
    header = msg->header;
    clusterPoints(input, header);
    
    old_cloud->clear();
    *old_cloud = *temp;
  }

  void TrackerNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input);
    
    geometry_msgs::msg::TransformStamped T;
    // Look up for the transformation between target_frame and source_frame
    try {
      T = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
      return;
    }
    // transform the pointcloud to map frame
    pcl_ros::transformPointCloud(*input, *input, T);
    std_msgs::msg::Header header;
    header = msg->header;
    clusterPoints(input, header);
    // clusterPoints(input, msg);

    // // Create the filtering object
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud (cloud);
    // sor.setLeafSize (0.05f, 0.05f, 0.01f);
    // sor.filter (*cloud_filtered);

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    if (!first_scan_)  *input += *old_cloud;

    // sor.setInputCloud (input);
    // sor.setMeanK (150);
    // sor.setStddevMulThresh (2.0);
    // sor.filter (*input);


  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracker::TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
