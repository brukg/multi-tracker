#include <tracker/tracker.hpp>


namespace tracker
{


  bool TrackerNode::isCompatible(const Eigen::VectorXd& pred, const Eigen::VectorXd& meas, double gate_threshold) 
  {
      double distance = (pred.head(2) - meas.head(2)).norm();
      return distance <= gate_threshold;
  }

  void TrackerNode::JCBB(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold, size_t pred_idx, std::vector<int>& current_association, std::vector<int>& best_association) 
  {
      if (pred_idx >= predictions.size()) {
          if (current_association.size() > best_association.size()) {
              best_association = current_association;
          }
          return;
      }

      // Mismatch (no association)
      JCBB(predictions, measurements, gate_threshold, pred_idx + 1, current_association, best_association);

      // Association
      for (size_t meas_idx = 0; meas_idx < measurements.size(); ++meas_idx) {
          if (isCompatible(predictions[pred_idx], measurements[meas_idx], gate_threshold) &&
              std::find(current_association.begin(), current_association.end(), meas_idx) == current_association.end()) {
              current_association.push_back(meas_idx);
              JCBB(predictions, measurements, gate_threshold, pred_idx + 1, current_association, best_association);
              current_association.pop_back();
          }
      }
  }

  std::vector<int> TrackerNode::JCBBAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold) 
  {
      std::vector<int> current_association;
      std::vector<int> best_association;
      JCBB(predictions, measurements, gate_threshold, 0, current_association, best_association);

      std::vector<int> associations(predictions.size(), -1);
      for (size_t i = 0; i < best_association.size(); ++i) {
          associations[i] = best_association[i];
      }
      // mrpt::math::CMatrixDouble& Z_observations_mean = Eigen:Map<mrpt::math::CMatrixDouble>(measurements.data(), measurements.size(), 2);
      // mrpt::math::CMatrixDouble& Z_observations_cov = Eigen:Map<mrpt::math::CMatrixDouble>(measurements.data(), measurements.size(), 2);

      return associations;
  }

  std::vector<int> TrackerNode::JPDAAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold) 
  {
      size_t n_predictions = predictions.size();
      size_t n_measurements = measurements.size();

      std::vector<int> associations(n_predictions, -1);

      Eigen::MatrixXd association_likelihoods = Eigen::MatrixXd::Zero(n_predictions, n_measurements + 1);

      for (size_t i = 0; i < n_predictions; ++i) {
          for (size_t j = 0; j < n_measurements; ++j) {
              double distance = (predictions[i].head(2) - measurements[j].head(2)).norm();
              if (distance <= gate_threshold) {
                // RCLCPP_INFO(this->get_logger(), "distance: %f", distance);
                association_likelihoods(i, j) = 1 + (1.0 / (distance + 1e-3));
              }
          }
          association_likelihoods(i, n_measurements) = 1; // Add extra column for mismatch
      }

      // Normalize likelihoods
      for (size_t i = 0; i < n_predictions; ++i) {
          double row_sum = association_likelihoods.row(i).sum();
          association_likelihoods.row(i) /= row_sum;
      }

      // Association
      for (size_t i = 0; i < n_predictions; ++i) {
          double max_likelihood = -std::numeric_limits<double>::infinity();
          size_t associated_index = -1; // Default to mismatch

          for (size_t j = 0; j <= n_measurements; ++j) {
              if (association_likelihoods(i, j) > max_likelihood) {
                  max_likelihood = association_likelihoods(i, j);
                  associated_index = j;
              }
          }
          // get the maximum in each column
          Eigen::MatrixXd::Index maxRow, maxCol;
          association_likelihoods.col(associated_index).maxCoeff(&maxRow, &maxCol);
          if (maxRow != i) {
              associated_index = n_measurements;
          }
          // RCLCPP_INFO(this->get_logger(), "associated_index: %d n_measurements: %d", associated_index, n_measurements);
          if (associated_index != n_measurements) { // If not a mismatch
              associations[i] = associated_index;
          } else {
              associations[i] = -1;
          }
      }

      return associations;
  }

  std::vector<int> TrackerNode::GreedyAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold) 
  {
      size_t n_predictions = predictions.size();
      size_t n_measurements = measurements.size();

      
    // get distances between predictions and measurements
    Eigen::MatrixXd dist(n_predictions,n_measurements);
    // RCLCPP_INFO(this->get_logger(), "measurements size: %d",n_measurements);

    // calculate distances between predictions and measurements and store in dist
    for (int i = 0; i < n_predictions; i++) {
      for (int j = 0; j < n_measurements; j++)
      {
        double dx = predictions[i](0) - measurements[j](0);
        double dy = predictions[i](1) - measurements[j](1);
        dist(i, j) = hypot(dx, dy);
      }
    }

    // find indices of minimum distance
    std::vector<int> min_dist_idx;
    std::vector<int> min_dist_vec;
    std::vector<int> meas_idx;
    for (int i = 0; i < n_predictions; i++) {
      int min_idx = -1;
      double min_dist =  max_dist_;
      for (int j = 0; j < n_measurements; j++) {
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

      return min_dist_idx;
  }

  // std::vector<int> TrackerNode::GreedyAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold) {
  //   size_t n_predictions = predictions.size();
  //   size_t n_measurements = measurements.size();

  //   // Calculate distances between predictions and measurements
  //   Eigen::MatrixXd dist(n_predictions, n_measurements);
  //   for (size_t i = 0; i < n_predictions; i++) {
  //     for (size_t j = 0; j < n_measurements; j++) {
  //       dist(i, j) = (predictions[i].head(2) - measurements[j].head(2)).norm();
  //     }
  //   }

  //   std::vector<int> min_dist_idx(n_predictions, -1);
  //   std::vector<int> min_dist_vec(n_predictions, max_dist_);
  //   std::vector<int> meas_idx;

  //   // Find indices of minimum distance
  //   for (size_t i = 0; i < n_predictions; i++) {
  //     for (size_t j = 0; j < n_measurements; j++) {
  //       if (dist(i, j) < min_dist_vec[i]) {
  //         auto found_iter = std::find(meas_idx.begin(), meas_idx.end(), j);
  //         if (found_iter == meas_idx.end()) {
  //           min_dist_vec[i] = dist(i, j);
  //           min_dist_idx[i] = j;
  //           meas_idx.push_back(j);
  //         }
  //       }
  //     }
  //   }

  //   // Apply gate threshold
  //   for (size_t i = 0; i < n_predictions; i++) {
  //     if (min_dist_vec[i] > gate_threshold) {
  //       min_dist_idx[i] = -1;
  //     }
  //   }

  //   return min_dist_idx;
  // }
  std::vector<int> TrackerNode::HungarianAssociate(const std::vector<Eigen::VectorXd>& predictions, const std::vector<Eigen::VectorXd>& measurements, double gate_threshold)
  {
    int num_predicted = predictions.size();
    int num_measured = measurements.size();
    Eigen::MatrixXf cost_matrix(num_predicted, num_measured);
    RCLCPP_INFO(this->get_logger(), "associateVectors: predictions.size(): %d, measurements.size(): %d", predictions.size(), measurements.size());
    // RCLCPP Info all the predictions members in structured style
    for (size_t i = 0; i < predictions.size(); i++) {
    }
    // RCLCPP Info all the measurements members in structured style
    for (size_t i = 0; i < measurements.size(); i++) {
    }
    // Compute cost matrix (negative distance) between predictions and measurements vectors
    dlib::matrix<int> dlib_cost_matrix(num_predicted, num_measured);

    for (size_t i = 0; i < num_predicted; i++) {
        for (size_t j = 0; j < num_measured; j++) {
            float dist = (predictions[i].head(2) - measurements[j].head(2)).norm();
            dlib_cost_matrix(i, j) = (dist <= max_dist_) ? static_cast<int>(std::round((max_dist_ - dist)*100)) : 0;//std::numeric_limits<int>::max();
        }
    }
    RCLCPP_INFO(this->get_logger(), "dlib_cost_matrix rows: %d, cols: %d", dlib_cost_matrix.nr(), dlib_cost_matrix.nc());

    // Solve assignment problem using the Hungarian algorithm
    std::vector<long>  assignment_matrix = dlib::max_cost_assignment(dlib_cost_matrix);
    // cast to int
    std::vector<int> associations(assignment_matrix.begin(), assignment_matrix.end());
    RCLCPP_INFO(this->get_logger(), "associations size: %d", associations.size());
    for (size_t i = 0; i < associations.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "associations[%d]: %d", i, associations[i]);
    }
    return associations;

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

        float displacement = euclideanDistance(previous_point, mean_shift);
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

        float distance = euclideanDistance(cloud->points[i], cloud->points[j]);
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
  float TrackerNode::euclideanDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
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
      float distance = euclideanDistance(point, neighbor);
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
} // namespace tracker