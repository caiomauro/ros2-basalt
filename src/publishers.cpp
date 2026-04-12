#include "basalt_wrapper/basalt_node.hpp"

#include <algorithm>

void BasaltNode::opticalFlowOutputLoop() {
  while (!shutting_down_) {
    basalt::OpticalFlowResult::Ptr flow_result;
    opt_flow_out_queue_.pop(flow_result);

    if (shutting_down_) {
      break;
    }
    if (!flow_result) {
      continue;
    }

    publishOpticalFlowDebug(flow_result);

    if (use_imu_) {
      std::unique_lock<std::mutex> lock(latest_imu_mutex_);
      latest_imu_cv_.wait(lock, [&] {
        return shutting_down_ || latest_imu_t_ns_ >= flow_result->t_ns;
      });
      if (shutting_down_) {
        break;
      }
    }

    vio_->vision_data_queue.push(flow_result);
  }
}

void BasaltNode::estimatorOutputLoop() {
  while (!shutting_down_) {
    basalt::PoseVelBiasState<double>::Ptr state;
    out_state_queue_.pop(state);

    if (shutting_down_) {
      break;
    }
    if (!state) {
      continue;
    }

    latest_state_ = state;
    publishOdometry(*state);
  }
}

void BasaltNode::publishOdometry(const basalt::PoseVelBiasState<double> &state) {
  nav_msgs::msg::Odometry odom{};
  odom.header.stamp = rclcpp::Time(state.t_ns);
  odom.header.frame_id = path_frame_id_;
  odom.child_frame_id = body_frame_id_;

  const Eigen::Vector3d translation = state.T_w_i.translation();
  const Eigen::Quaterniond orientation(state.T_w_i.unit_quaternion());
  const Eigen::Vector3d velocity = state.vel_w_i;

  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  {
    std::lock_guard<std::mutex> lock(latest_imu_mutex_);
    angular_velocity = latest_gyro_body_;
  }

  odom.pose.pose.position.x = translation.x();
  odom.pose.pose.position.y = translation.y();
  odom.pose.pose.position.z = translation.z();
  odom.pose.pose.orientation.w = orientation.w();
  odom.pose.pose.orientation.x = orientation.x();
  odom.pose.pose.orientation.y = orientation.y();
  odom.pose.pose.orientation.z = orientation.z();

  odom.twist.twist.linear.x = velocity.x();
  odom.twist.twist.linear.y = velocity.y();
  odom.twist.twist.linear.z = velocity.z();
  odom.twist.twist.angular.x = angular_velocity.x();
  odom.twist.twist.angular.y = angular_velocity.y();
  odom.twist.twist.angular.z = angular_velocity.z();

  const auto pose_cov = defaultCovariance(0.05, 0.02);
  const auto twist_cov = defaultCovariance(0.05, 0.02);
  odom.pose.covariance = pose_cov;
  odom.twist.covariance = twist_cov;

  odom_pub_->publish(odom);
  recordTrajectory(state);
  publishPath(state);
  ++odom_published_;
  if (odom_published_ % 50 == 0) {
    RCLCPP_INFO(get_logger(),
                "published odometry count=%zu t_ns=%lld path_points=%zu",
                odom_published_, static_cast<long long>(state.t_ns),
                path_msg_.poses.size());
  }
}

void BasaltNode::recordTrajectory(const basalt::PoseVelBiasState<double> &state) {
  if (!trajectory_file_.is_open()) {
    return;
  }

  std::lock_guard<std::mutex> lock(trajectory_file_mutex_);
  if (state.t_ns == last_recorded_state_t_ns_) {
    return;
  }
  last_recorded_state_t_ns_ = state.t_ns;

  const Eigen::Vector3d translation = state.T_w_i.translation();
  const Eigen::Quaterniond orientation(state.T_w_i.unit_quaternion());

  trajectory_file_.setf(std::ios::fixed);
  trajectory_file_.precision(9);
  trajectory_file_ << static_cast<double>(state.t_ns) / 1e9 << " "
                   << translation.x() << " " << translation.y() << " "
                   << translation.z() << " " << orientation.x() << " "
                   << orientation.y() << " " << orientation.z() << " "
                   << orientation.w() << "\n";
}

void BasaltNode::publishPath(const basalt::PoseVelBiasState<double> &state) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = rclcpp::Time(state.t_ns);
  pose.header.frame_id = path_frame_id_;

  const Eigen::Vector3d raw_translation = state.T_w_i.translation();
  const Eigen::Quaterniond raw_orientation(state.T_w_i.unit_quaternion());

  {
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    latest_raw_pose_translation_ = raw_translation;
    latest_raw_pose_orientation_ = raw_orientation;
  }

  pose.pose.position.x = raw_translation.x();
  pose.pose.position.y = raw_translation.y();
  pose.pose.position.z = raw_translation.z();
  pose.pose.orientation.w = raw_orientation.w();
  pose.pose.orientation.x = raw_orientation.x();
  pose.pose.orientation.y = raw_orientation.y();
  pose.pose.orientation.z = raw_orientation.z();

  path_msg_.header.stamp = pose.header.stamp;
  path_msg_.poses.push_back(pose);

  geometry_msgs::msg::Point32 pose_point;
  pose_point.x = static_cast<float>(raw_translation.x());
  pose_point.y = static_cast<float>(raw_translation.y());
  pose_point.z = static_cast<float>(raw_translation.z());
  pose_cloud_msg_.header.stamp = pose.header.stamp;
  pose_cloud_msg_.points.push_back(pose_point);

  if (max_path_length_ > 0 &&
      static_cast<int>(path_msg_.poses.size()) > max_path_length_) {
    const auto excess = path_msg_.poses.size() - max_path_length_;
    path_msg_.poses.erase(path_msg_.poses.begin(),
                          path_msg_.poses.begin() +
                              static_cast<std::ptrdiff_t>(excess));
    pose_cloud_msg_.points.erase(
        pose_cloud_msg_.points.begin(),
        pose_cloud_msg_.points.begin() +
            static_cast<std::ptrdiff_t>(
                std::min(excess, pose_cloud_msg_.points.size())));
  }

  pose_pub_->publish(pose);
  path_pub_->publish(path_msg_);
  pose_cloud_pub_->publish(pose_cloud_msg_);
}
