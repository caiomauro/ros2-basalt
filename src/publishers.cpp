// Copyright 2026 Caio Mauro
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Caio Mauro nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "basalt_wrapper/basalt_node.hpp"

#include <algorithm>

void BasaltNode::opticalFlowOutputLoop() {
  while (true) {
    basalt::OpticalFlowResult::Ptr flow_result;
    opt_flow_out_queue_.pop(flow_result);

    if (!flow_result) {
      break;
    }
    if (shutting_down_) {
      // Keep draining until Basalt optical flow's terminal null sentinel so
      // its worker cannot block on a full output queue during shutdown.
      continue;
    }

    // Forward estimator-critical data first. Overlay construction performs
    // image conversion and drawing and must never stall the VIO input path.
    vio_->vision_data_queue.push(flow_result);

    if (diagnostics_log_file_.is_open()) {
      const size_t left_tracks = flow_result->observations.empty()
                                     ? 0
                                     : flow_result->observations[0].size();
      const size_t right_tracks = flow_result->observations.size() < 2
                                      ? 0
                                      : flow_result->observations[1].size();
      std::lock_guard<std::mutex> lock(diagnostics_log_mutex_);
      diagnostics_log_file_ << "optical_flow," << flow_result->t_ns << ","
                            << steadyNowNs() << "," << left_tracks << ","
                            << right_tracks << ","
                            << vio_->vision_data_queue.size() << ","
                            << vio_->imu_data_queue.size() << ",0,0\n";
    }

    const int64_t min_debug_period_ns = static_cast<int64_t>(
        1e9 / std::max(0.1, debug_visual_rate_hz_));
    if (publish_debug_visuals_ &&
        (last_debug_visual_t_ns_ == std::numeric_limits<int64_t>::min() ||
         flow_result->t_ns - last_debug_visual_t_ns_ >= min_debug_period_ns)) {
      last_debug_visual_t_ns_ = flow_result->t_ns;
      publishOpticalFlowDebug(flow_result);
    }
  }
}

void BasaltNode::estimatorOutputLoop() {
  while (true) {
    basalt::PoseVelBiasState<double>::Ptr state;
    out_state_queue_.pop(state);

    if (!state) {
      break;
    }
    if (shutting_down_) {
      continue;
    }

    const double accel_bias_norm = state->bias_accel.norm();
    const double gyro_bias_norm = state->bias_gyro.norm();
    const double speed_norm = state->vel_w_i.norm();
    const bool finite = state->T_w_i.translation().allFinite() &&
                        state->vel_w_i.allFinite() &&
                        state->bias_accel.allFinite() &&
                        state->bias_gyro.allFinite();
    if (estimator_last_health_t_ns_ > 0 && state->t_ns > estimator_last_health_t_ns_) {
      const double dt = static_cast<double>(state->t_ns - estimator_last_health_t_ns_) * 1e-9;
      const double slope = (accel_bias_norm - estimator_last_accel_bias_norm_) /
                           std::max(1e-6, dt);
      estimator_accel_bias_slope_ewma_mps3_ =
          0.90 * estimator_accel_bias_slope_ewma_mps3_ + 0.10 * slope;
    }
    estimator_last_health_t_ns_ = state->t_ns;
    estimator_last_accel_bias_norm_ = accel_bias_norm;
    const bool predictive_warning =
        finite && accel_bias_norm >= health_warning_accel_bias_mps2_ &&
        estimator_accel_bias_slope_ewma_mps3_ >=
            health_warning_accel_bias_slope_mps3_;
    estimator_warning_consecutive_states_ =
        predictive_warning ? estimator_warning_consecutive_states_ + 1
                           : std::max(0, estimator_warning_consecutive_states_ - 1);
    const bool predictive_alert =
        estimator_warning_consecutive_states_ >=
        health_warning_consecutive_states_;
    if (estimator_warning_consecutive_states_ ==
        health_warning_consecutive_states_) {
      RCLCPP_WARN(
          get_logger(),
          "Basalt predictive health warning: accel bias is rising accel_bias=%.6f slope=%.6f; continuing unless the validated hard health limit is crossed",
          accel_bias_norm, estimator_accel_bias_slope_ewma_mps3_);
    }
    const bool plausible =
        finite && accel_bias_norm <= health_max_accel_bias_mps2_ &&
        gyro_bias_norm <= health_max_gyro_bias_rad_s_ &&
        speed_norm <= health_max_speed_mps_;
    estimator_unhealthy_consecutive_states_ =
        plausible ? std::max(0, estimator_unhealthy_consecutive_states_ - 1)
                  : estimator_unhealthy_consecutive_states_ + 1;
    if (!estimator_health_faulted_.load() &&
        estimator_unhealthy_consecutive_states_ >=
            health_fault_consecutive_states_) {
      estimator_health_faulted_.store(true);
      RCLCPP_ERROR(
          get_logger(),
          "Basalt estimator health failure: suppressing invalid odometry accel_bias=%.6f accel_bias_slope=%.6f gyro_bias=%.6f speed=%.3f predictive=%s finite=%s consecutive=%d",
          accel_bias_norm, estimator_accel_bias_slope_ewma_mps3_,
          gyro_bias_norm, speed_norm,
          predictive_alert ? "true" : "false",
          finite ? "true" : "false", estimator_unhealthy_consecutive_states_);
    }

    if (estimator_health_faulted_.load()) {
      estimator_healthy_recovery_states_ = plausible
          ? estimator_healthy_recovery_states_ + 1
          : 0;
      if (estimator_healthy_recovery_states_ >=
          health_recovery_consecutive_states_) {
        estimator_health_faulted_.store(false);
        estimator_unhealthy_consecutive_states_ = 0;
        estimator_healthy_recovery_states_ = 0;
        RCLCPP_WARN(
            get_logger(),
            "Basalt estimator health recovered after %d consecutive plausible states; resuming odometry",
            health_recovery_consecutive_states_);
      }
    } else {
      estimator_healthy_recovery_states_ = 0;
    }

    {
      std::lock_guard<std::mutex> lock(latest_state_mutex_);
      latest_state_ = state;
    }
    if (!estimator_health_faulted_.load()) {
      publishOdometry(*state);
    }
    if (diagnostics_log_file_.is_open()) {
      const auto &p = state->T_w_i.translation();
      std::lock_guard<std::mutex> lock(diagnostics_log_mutex_);
      diagnostics_log_file_ << "state," << state->t_ns << "," << steadyNowNs()
                            << "," << p.x() << "," << p.y() << "," << p.z()
                            << "," << state->bias_gyro.norm() << ","
                            << state->bias_accel.norm() << ","
                            << out_state_queue_.size() << "\n";
      diagnostics_log_file_ << "health," << state->t_ns << "," << steadyNowNs()
                            << "," << accel_bias_norm << "," << gyro_bias_norm
                            << "," << speed_norm << ","
                            << estimator_accel_bias_slope_ewma_mps3_ << ","
                            << estimator_warning_consecutive_states_ << ","
                            << (estimator_health_faulted_.load() ? 1 : 0) << "\n";
    }
  }
}

void BasaltNode::publishOdometry(const basalt::PoseVelBiasState<double> &state) {
  const double min_publish_period_ns = 1e9 / std::max(1e-3, publish_rate_hz_);
  if (last_odom_publish_t_ns_ != std::numeric_limits<int64_t>::min() &&
      static_cast<double>(state.t_ns - last_odom_publish_t_ns_) <
          min_publish_period_ns) {
    return;
  }
  last_odom_publish_t_ns_ = state.t_ns;

  nav_msgs::msg::Odometry odom{};
  odom.header.stamp = rclcpp::Time(state.t_ns);
  odom.header.frame_id = path_frame_id_;
  odom.child_frame_id = body_frame_id_;

  Eigen::Vector3d translation;
  Eigen::Vector3d velocity;
  startupAdjustedMotion(state.T_w_i.translation(), state.vel_w_i, translation,
                        velocity);
  const Eigen::Quaterniond orientation(state.T_w_i.unit_quaternion());

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
  latest_published_state_t_ns_.store(state.t_ns);
  recordTrajectory(state);
  publishPath(state);
  ++odom_published_;
  if (odom_published_ % 50 == 0) {
    const int64_t lag_ns = std::max<int64_t>(0, last_image_t_ns_ - state.t_ns);
    RCLCPP_INFO(get_logger(),
                "published odometry count=%zu t_ns=%lld path_points=%zu lag_ms=%.1f",
                odom_published_, static_cast<long long>(state.t_ns),
                path_msg_.poses.size(), static_cast<double>(lag_ns) / 1e6);
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

  Eigen::Vector3d translation;
  Eigen::Vector3d velocity;
  startupAdjustedMotion(state.T_w_i.translation(), state.vel_w_i, translation,
                        velocity);
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
  Eigen::Vector3d output_translation;
  Eigen::Vector3d output_velocity;
  startupAdjustedMotion(raw_translation, state.vel_w_i, output_translation,
                        output_velocity);
  const Eigen::Quaterniond raw_orientation(state.T_w_i.unit_quaternion());

  {
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    latest_raw_pose_translation_ = raw_translation;
    latest_raw_pose_orientation_ = raw_orientation;
  }

  pose.pose.position.x = output_translation.x();
  pose.pose.position.y = output_translation.y();
  pose.pose.position.z = output_translation.z();
  pose.pose.orientation.w = raw_orientation.w();
  pose.pose.orientation.x = raw_orientation.x();
  pose.pose.orientation.y = raw_orientation.y();
  pose.pose.orientation.z = raw_orientation.z();

  path_msg_.header.stamp = pose.header.stamp;
  path_msg_.poses.push_back(pose);

  geometry_msgs::msg::Point32 pose_point;
  pose_point.x = static_cast<float>(output_translation.x());
  pose_point.y = static_cast<float>(output_translation.y());
  pose_point.z = static_cast<float>(output_translation.z());
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
  if (odom_published_ % static_cast<size_t>(path_publish_stride_) == 0) {
    path_pub_->publish(path_msg_);
    pose_cloud_pub_->publish(pose_cloud_msg_);
    ++path_published_;
    if (path_published_ % 50 == 0) {
      RCLCPP_INFO(get_logger(),
                  "published path bundle count=%zu poses=%zu cloud_points=%zu stride=%d",
                  path_published_, path_msg_.poses.size(),
                  pose_cloud_msg_.points.size(), path_publish_stride_);
    }
  }
}
