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

void BasaltNode::applyStartupImuCompensation(Eigen::Vector3d &gyro,
                                             Eigen::Vector3d &accel,
                                             int64_t input_t_ns) {
  if (!imu_startup_calibration_enabled_) {
    return;
  }

  if (imu_startup_calibration_start_t_ns_ == 0) {
    imu_startup_calibration_start_t_ns_ = input_t_ns;
  }

  imu_startup_gyro_sum_ += gyro;
  imu_startup_accel_sum_ += accel;
  imu_startup_gyro_sq_sum_ += gyro.cwiseProduct(gyro);
  imu_startup_accel_sq_sum_ += accel.cwiseProduct(accel);
  ++imu_startup_calibration_samples_;

  const double sample_count =
      static_cast<double>(std::max<size_t>(imu_startup_calibration_samples_, 1));
  const Eigen::Vector3d mean_gyro = imu_startup_gyro_sum_ / sample_count;
  const Eigen::Vector3d mean_accel = imu_startup_accel_sum_ / sample_count;

  const Eigen::Vector3d provisional_gyro_bias = mean_gyro;

  if (!imu_startup_bias_ready_) {
    const double elapsed_sec =
        static_cast<double>(input_t_ns - imu_startup_calibration_start_t_ns_) *
        1e-9;
    if (elapsed_sec >= imu_startup_calibration_sec_ &&
        static_cast<int>(imu_startup_calibration_samples_) >=
            imu_startup_min_samples_) {
      const Eigen::Vector3d gyro_var =
          (imu_startup_gyro_sq_sum_ / sample_count -
           mean_gyro.cwiseProduct(mean_gyro))
              .cwiseMax(0.0);
      const Eigen::Vector3d accel_var =
          (imu_startup_accel_sq_sum_ / sample_count -
           mean_accel.cwiseProduct(mean_accel))
              .cwiseMax(0.0);
      const Eigen::Vector3d gyro_std = gyro_var.cwiseSqrt();
      const Eigen::Vector3d accel_std = accel_var.cwiseSqrt();
      const bool stationary =
          gyro_std.maxCoeff() <= imu_startup_max_gyro_std_ &&
          accel_std.maxCoeff() <= imu_startup_max_accel_std_ &&
          std::abs(mean_accel.norm() - imu_expected_gravity_mps2_) <=
              imu_startup_gravity_tolerance_;

      if (stationary) {
        imu_startup_gyro_bias_ = provisional_gyro_bias;
        // Preserve the measured gravity direction.  The vehicle may rest at a
        // few degrees of roll/pitch on the terrain, so treating stationary X/Y
        // acceleration as sensor bias corrupts Basalt's initial attitude.
        // Only remove the small radial error in gravity magnitude.
        const double mean_accel_norm = mean_accel.norm();
        if (mean_accel_norm > 1e-9) {
          imu_startup_accel_bias_ =
              mean_accel *
              ((mean_accel_norm - imu_expected_gravity_mps2_) /
               mean_accel_norm);
        } else {
          imu_startup_accel_bias_.setZero();
        }
        imu_startup_bias_ready_ = true;
        startup_stationary_accel_reference_ =
            mean_accel - imu_startup_accel_bias_;
        // The public zero-motion clamp/rebase is always active through
        // takeoff detection. Internal global-position factors are optional:
        // repeated pseudo-measurements made the simulator's bias state
        // unobservable and must not be enabled by default.
        startup_stationary_phase_active_.store(true);
        RCLCPP_INFO(
            get_logger(),
            "locked stationary IMU initialization after %.2fs samples=%zu gyro_bias=[%.6f %.6f %.6f] accel_bias=[%.6f %.6f %.6f] raw_gravity_body=[%.6f %.6f %.6f] gyro_std_max=%.6f accel_std_max=%.6f",
            elapsed_sec, imu_startup_calibration_samples_,
            imu_startup_gyro_bias_.x(), imu_startup_gyro_bias_.y(),
            imu_startup_gyro_bias_.z(), imu_startup_accel_bias_.x(),
            imu_startup_accel_bias_.y(), imu_startup_accel_bias_.z(),
            mean_accel.x(), mean_accel.y(), mean_accel.z(),
            gyro_std.maxCoeff(), accel_std.maxCoeff());
      } else {
        RCLCPP_WARN(
            get_logger(),
            "IMU startup window was not stationary; restarting calibration gyro_std_max=%.6f accel_std_max=%.6f accel_norm=%.6f",
            gyro_std.maxCoeff(), accel_std.maxCoeff(), mean_accel.norm());
        imu_startup_calibration_start_t_ns_ = input_t_ns;
        imu_startup_calibration_samples_ = 0;
        imu_startup_gyro_sum_.setZero();
        imu_startup_accel_sum_.setZero();
        imu_startup_gyro_sq_sum_.setZero();
        imu_startup_accel_sq_sum_.setZero();
      }
    }
  }

  if (imu_startup_bias_ready_) {
    gyro -= imu_startup_gyro_bias_;
    accel -= imu_startup_accel_bias_;

    // Keep the zero-motion anchor only through the initial stationary phase.
    // A few isolated simulated IMU spikes must not release it, while motor
    // startup / takeoff produces sustained angular rate or a clear change in
    // the gravity vector and permanently releases the constraint.
    if (startup_stationary_phase_active_) {
      const bool moving =
          gyro.norm() > startup_motion_gyro_threshold_rad_s_ ||
          (accel - startup_stationary_accel_reference_).norm() >
              startup_motion_accel_delta_threshold_mps2_;
      startup_motion_consecutive_samples_ =
          moving ? startup_motion_consecutive_samples_ + 1
                 : std::max(0, startup_motion_consecutive_samples_ - 1);
      if (startup_motion_consecutive_samples_ >= 10) {
        Eigen::Vector3d raw_translation = Eigen::Vector3d::Zero();
        Eigen::Vector3d raw_velocity = Eigen::Vector3d::Zero();
        {
          std::lock_guard<std::mutex> lock(latest_state_mutex_);
          if (latest_state_) {
            raw_translation = latest_state_->T_w_i.translation();
            raw_velocity = latest_state_->vel_w_i;
          }
          startup_rebase_translation_ = raw_translation;
          startup_rebase_velocity_ = raw_velocity;
          startup_rebase_ready_ = true;
        }
        startup_stationary_phase_active_.store(false);
        RCLCPP_INFO(
            get_logger(),
            "released startup stationary clamp after sustained motion gyro_norm=%.4f accel_delta=%.4f anchors=%zu position_rebase=[%.3f %.3f %.3f] velocity_rebase=[%.3f %.3f %.3f]",
            gyro.norm(),
            (accel - startup_stationary_accel_reference_).norm(),
            startup_anchors_added_, raw_translation.x(), raw_translation.y(),
            raw_translation.z(), raw_velocity.x(), raw_velocity.y(),
            raw_velocity.z());
      }
    }
  }
}

void BasaltNode::maybeAddStartupStationaryAnchor() {
  if (!startup_stationary_anchor_enabled_ ||
      !startup_stationary_phase_active_.load() || !vio_) {
    return;
  }

  const int64_t state_t_ns = latest_published_state_t_ns_.load();
  if (state_t_ns <= 0) return;
  const int64_t interval_ns = static_cast<int64_t>(
      1e9 / startup_stationary_anchor_rate_hz_);
  if (last_startup_anchor_t_ns_ > 0 &&
      state_t_ns - last_startup_anchor_t_ns_ < interval_ns) {
    return;
  }

  auto measurement = std::make_shared<basalt::GlobalPositionMeasurement>();
  measurement->t_ns = state_t_ns;
  measurement->position.setZero();
  measurement->covariance = Eigen::Matrix3d::Identity() *
                            (startup_stationary_anchor_std_m_ *
                             startup_stationary_anchor_std_m_);
  vio_->addGlobalPositionMeasurement(measurement);
  last_startup_anchor_t_ns_ = state_t_ns;
  ++startup_anchors_added_;
  if (startup_anchors_added_ <= 3 || startup_anchors_added_ % 25 == 0) {
    RCLCPP_INFO(get_logger(),
                "added startup stationary position anchor #%zu t_ns=%lld",
                startup_anchors_added_, static_cast<long long>(state_t_ns));
  }
}

void BasaltNode::startupAdjustedMotion(
    const Eigen::Vector3d &raw_translation,
    const Eigen::Vector3d &raw_velocity, Eigen::Vector3d &translation,
    Eigen::Vector3d &velocity) {
  if (startup_stationary_phase_active_.load()) {
    translation.setZero();
    velocity.setZero();
    return;
  }

  std::lock_guard<std::mutex> lock(latest_state_mutex_);
  translation = raw_translation;
  velocity = raw_velocity;
  if (startup_rebase_ready_) {
    translation -= startup_rebase_translation_;
    velocity -= startup_rebase_velocity_;
  }
}

void BasaltNode::storeLatestImuDebug(const Eigen::Vector3d &raw_gyro,
                                     const Eigen::Vector3d &raw_accel,
                                     const Eigen::Vector3d &basalt_gyro,
                                     const Eigen::Vector3d &basalt_accel,
                                     int64_t input_t_ns) {
  std::lock_guard<std::mutex> lock(latest_imu_mutex_);
  latest_raw_imu_gyro_ = raw_gyro;
  latest_raw_imu_accel_ = raw_accel;
  latest_basalt_gyro_ = basalt_gyro;
  latest_basalt_accel_ = basalt_accel;
  latest_gyro_body_ = basalt_gyro;
  latest_imu_t_ns_ = input_t_ns;
}

void BasaltNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (shutting_down_ || !vio_) {
    return;
  }

  const int64_t input_t_ns =
      use_header_timestamps_ ? rosTimeToNs(msg->header.stamp) : steadyNowNs();
  const uint64_t seq = ++next_imu_source_seq_;
  if (seq <= 5) {
    RCLCPP_INFO(
        get_logger(),
        "imu arrival seq=%llu t_ns=%lld ang=[%.6f %.6f %.6f] acc=[%.6f %.6f %.6f]",
        static_cast<unsigned long long>(seq),
        static_cast<long long>(input_t_ns), msg->angular_velocity.x,
        msg->angular_velocity.y, msg->angular_velocity.z,
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z);
  }
  enqueueImuInput(PendingImuInput{input_t_ns, msg, seq, imu_topic_});
}

void BasaltNode::globalPositionCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (shutting_down_ || !vio_ || !use_global_position_factors_) return;

  const int64_t t_ns = rosTimeToNs(msg->header.stamp);
  Eigen::Vector3d position(msg->pose.pose.position.x,
                           msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
  {
    // VPC observes the public, startup-rebased odometry frame. Convert its
    // target back into Basalt's internal frame before creating the factor.
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    if (startup_rebase_ready_) position += startup_rebase_translation_;
  }
  Eigen::Matrix3d covariance;
  covariance << msg->pose.covariance[0], msg->pose.covariance[1],
      msg->pose.covariance[2], msg->pose.covariance[6],
      msg->pose.covariance[7], msg->pose.covariance[8],
      msg->pose.covariance[12], msg->pose.covariance[13],
      msg->pose.covariance[14];

  if (t_ns <= 0 || !position.allFinite() || !covariance.allFinite() ||
      covariance.diagonal().minCoeff() <= 0.0) {
    ++global_positions_rejected_;
    RCLCPP_WARN(get_logger(),
                "rejected invalid VPC global-position factor count=%zu",
                global_positions_rejected_);
    return;
  }

  auto measurement = std::make_shared<basalt::GlobalPositionMeasurement>();
  measurement->t_ns = t_ns;
  measurement->position = position;
  measurement->covariance = covariance;
  vio_->addGlobalPositionMeasurement(measurement);
  ++global_positions_received_;

  if (diagnostics_log_file_.is_open()) {
    std::lock_guard<std::mutex> lock(diagnostics_log_mutex_);
    diagnostics_log_file_ << "global_position," << t_ns << ","
                          << steadyNowNs() << "," << position.x() << ","
                          << position.y() << "," << position.z() << ","
                          << covariance(0, 0) << "," << covariance(1, 1)
                          << "," << covariance(2, 2) << "\n";
  }
}
