#include "tracker/tracker.hpp"
namespace tracker
{
  void TrackerNode::publishMarkers()
  {

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->now();
    marker.ns = "objects";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
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
    velocity.action = visualization_msgs::msg::Marker::ADD;
    velocity.scale.x = 0.1;
    velocity.scale.y = 0.1;
    velocity.scale.z = 0.05;
    velocity.color.a = 1.0;
    velocity.color.r = 0.0;
    velocity.color.g = 1.0;
    velocity.color.b = 0.0;
    velocity_array.markers.clear();

    visualization_msgs::msg::MarkerArray trajectories;
    visualization_msgs::msg::Marker trajecotry;
    trajecotry.header.frame_id = map_frame_;
    trajecotry.header.stamp = this->now();
    trajecotry.ns = "trajectory";
    trajecotry.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajecotry.action = visualization_msgs::msg::Marker::ADD;
    trajecotry.scale.x = 0.05;
    trajecotry.color.a = 1.0;
    trajecotry.color.r = 0.0;
    trajecotry.color.g = 1.0;
    trajecotry.color.b = 0.0;
    trajecotry.points.clear();

    visualization_msgs::msg::MarkerArray particles;
    visualization_msgs::msg::Marker point_particles;  
    point_particles.header.frame_id = map_frame_;
    point_particles.header.stamp = this->now();
    point_particles.ns = "point_particles";
    point_particles.type = visualization_msgs::msg::Marker::POINTS;
    point_particles.action = visualization_msgs::msg::Marker::ADD;
    point_particles.scale.x = 0.01;
    point_particles.scale.y = 0.01;
    point_particles.scale.z = 0.01;
    point_particles.color.a = 1.0;
    point_particles.color.r = 1.0;
    point_particles.color.g = 0.0;
    point_particles.color.b = 0.0;
    particles.markers.clear();
    point_particles.points.clear();

    
    // marker_pub_->publish(marker_array); // marker publish
    // marker_pub_->publish(velocity_array); // velocity publish
    // rviz_interface.deleteAllMarkers();
    // objects id push
    std_msgs::msg::Int32MultiArray objects_id_;
    objects_id_.data.clear();
    
    for (int i = 0; i < objects_.size(); i++) {
      float x_, y_, vx_, vy_, R_; // state of object
      x_ = objects_[i].getState()(0);
      y_ = objects_[i].getState()(1);
      vx_ = objects_[i].getState()(2);
      vy_ = objects_[i].getState()(3);
      R_ = objects_[i].getRadius();

      marker.id = i;//objects_[i].getIdentifier();
      marker.pose.position.x = x_;
      marker.pose.position.y = y_;

      marker.scale.x = R_+0.001;
      marker.scale.y = R_+0.001;
      marker.scale.z = 0.1;

      marker_array.markers.push_back(marker);

      // obstacle states
      state_array.data.push_back(x_);
      state_array.data.push_back(y_);
      state_array.data.push_back(vx_);
      state_array.data.push_back(vy_);

      // velocity
      velocity.id = i;//objects_[i].getIdentifier();
      velocity.pose.position.x = objects_[i].getState()(0);
      velocity.pose.position.y = objects_[i].getState()(1);
      velocity.pose.position.z = 0.1;
      double theta = atan2(objects_[i].getState()(3), objects_[i].getState()(2));
      tf2::Quaternion qt;
      qt.setRPY(0, 0, theta);
      velocity.pose.orientation.x = qt.x();
      velocity.pose.orientation.y = qt.y();
      velocity.pose.orientation.z = qt.z();
      velocity.pose.orientation.w = qt.w();
      velocity.scale.x = hypot(objects_[i].getState()(2), objects_[i].getState()(3)) +0.0001;
      velocity_array.markers.push_back(velocity);

      // objects id push
      objects_id_.data.push_back(objects_[i].getIdentifier());


      // particles
      geometry_msgs::msg::Point p;
      Eigen::MatrixXd point_praticle = objects_[i].getEnsembles();
      point_particles.id = i;
      for (int j = 0; j < point_praticle.cols(); j++) {
        p.x = point_praticle(0, j);
        p.y = point_praticle(1, j);
        p.z = 0.1;
        point_particles.points.push_back(p);
      }
      particles.markers.push_back(point_particles);

      // trajectory
      trajecotry.id = i;
      trajecotry.points.clear();
      for (int j = 0; j < int(3/0.1); j++) {
        p.x = x_ + vx_*j*0.1;
        p.y = y_ + vy_*j*0.1;
        p.z = 0.1;
        trajecotry.points.push_back(p);
      }
      trajectories.markers.push_back(trajecotry);
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
    marker_pub_->publish(particles);
    marker_pub_->publish(trajectories);

  }


}