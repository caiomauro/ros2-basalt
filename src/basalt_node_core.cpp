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

#include <cereal/archives/json.hpp>

#include <basalt/serialization/headers_serialization.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <openssl/sha.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

BasaltNode::BasaltNode() : rclcpp::Node("basalt_node") {
  left_image_topic_ =
      declare_parameter<std::string>("left_image_topic", "/camera/image_raw");
  right_image_topic_ = declare_parameter<std::string>("right_image_topic", "");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
  calib_path_ = declare_parameter<std::string>("calib_path", "");
  config_path_ = declare_parameter<std::string>("config_path", "");
  trajectory_output_path_ =
      declare_parameter<std::string>("trajectory_output_path", "");
  ingest_log_path_ = declare_parameter<std::string>("ingest_log_path", "");
  diagnostics_log_path_ =
      declare_parameter<std::string>("diagnostics_log_path", "");
  marg_data_output_path_ =
      declare_parameter<std::string>("marg_data_output_path", "");
  accepted_input_bag_path_ =
      declare_parameter<std::string>("accepted_input_bag_path", "");
  accepted_input_bag_cache_mb_ =
      declare_parameter<int>("accepted_input_bag_cache_mb", 256);
  global_position_topic_ = declare_parameter<std::string>(
      "global_position_topic", "/geo/vio_position_measurement");
  input_mode_ = declare_parameter<std::string>("input_mode", "ros_topics");
  bag_uri_ = declare_parameter<std::string>("bag_uri", "");
  bag_storage_id_ = declare_parameter<std::string>("bag_storage_id", "sqlite3");
  bag_start_time_ns_ = declare_parameter<int64_t>("bag_start_time_ns", 0);
  bag_preserve_record_order_ =
      declare_parameter<bool>("bag_preserve_record_order", false);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
  path_frame_id_ =
      declare_parameter<std::string>("path_frame_id", "basalt_world");
  body_frame_id_ =
      declare_parameter<std::string>("body_frame_id", "basalt_body");
  max_path_length_ = declare_parameter<int>("max_path_length", 10000);
  path_publish_stride_ = declare_parameter<int>("path_publish_stride", 10);
  use_imu_ = declare_parameter<bool>("use_imu", true);
  use_double_ = declare_parameter<bool>("use_double", true);
  use_camera_info_calibration_ =
      declare_parameter<bool>("use_camera_info_calibration", false);
  camera_model_ = declare_parameter<std::string>("camera_model", "pinhole");
  use_header_timestamps_ =
      declare_parameter<bool>("use_header_timestamps", true);
  publish_debug_visuals_ =
      declare_parameter<bool>("publish_debug_visuals", false);
  publish_replay_inputs_ =
      declare_parameter<bool>("publish_replay_inputs", false);
  debug_visual_rate_hz_ =
      declare_parameter<double>("debug_visual_rate_hz", 5.0);
  max_stereo_input_rate_hz_ =
      declare_parameter<double>("max_stereo_input_rate_hz", 0.0);
  health_max_accel_bias_mps2_ =
      declare_parameter<double>("health_max_accel_bias_mps2", 0.75);
  health_max_gyro_bias_rad_s_ =
      declare_parameter<double>("health_max_gyro_bias_rad_s", 0.03);
  health_max_speed_mps_ =
      declare_parameter<double>("health_max_speed_mps", 45.0);
  health_warning_accel_bias_mps2_ = declare_parameter<double>(
      "health_warning_accel_bias_mps2", 0.35);
  health_warning_accel_bias_slope_mps3_ = declare_parameter<double>(
      "health_warning_accel_bias_slope_mps3", 0.03);
  health_fault_consecutive_states_ =
      declare_parameter<int>("health_fault_consecutive_states", 3);
  health_warning_consecutive_states_ =
      declare_parameter<int>("health_warning_consecutive_states", 20);
  health_recovery_consecutive_states_ =
      declare_parameter<int>("health_recovery_consecutive_states", 30);
  use_global_position_factors_ =
      declare_parameter<bool>("use_global_position_factors", true);
  imu_startup_calibration_enabled_ =
      declare_parameter<bool>("imu_startup_calibration_enabled", true);
  startup_stationary_anchor_enabled_ =
      declare_parameter<bool>("startup_stationary_anchor_enabled", false);
  image_width_ = declare_parameter<int>("image_width", 1280);
  image_height_ = declare_parameter<int>("image_height", 960);
  stereo_sync_queue_size_ =
      declare_parameter<int>("stereo_sync_queue_size", 1000);
  max_pending_imu_inputs_ =
      declare_parameter<int>("max_pending_imu_inputs", 2000);
  max_pending_image_inputs_ =
      declare_parameter<int>("max_pending_image_inputs", 250);
  fx_ = declare_parameter<double>("fx", 539.9363327026367);
  fy_ = declare_parameter<double>("fy", 539.9363708496094);
  cx_ = declare_parameter<double>("cx", 640.0);
  cy_ = declare_parameter<double>("cy", 480.0);
  imu_startup_calibration_sec_ =
      declare_parameter<double>("imu_startup_calibration_sec", 3.0);
  imu_expected_gravity_mps2_ =
      declare_parameter<double>("imu_expected_gravity_mps2", 9.81);
  imu_startup_max_gyro_std_ =
      declare_parameter<double>("imu_startup_max_gyro_std", 0.02);
  imu_startup_max_accel_std_ =
      declare_parameter<double>("imu_startup_max_accel_std", 0.25);
  imu_startup_gravity_tolerance_ =
      declare_parameter<double>("imu_startup_gravity_tolerance", 0.75);
  startup_stationary_anchor_std_m_ =
      declare_parameter<double>("startup_stationary_anchor_std_m", 0.03);
  startup_stationary_anchor_rate_hz_ =
      declare_parameter<double>("startup_stationary_anchor_rate_hz", 5.0);
  startup_motion_gyro_threshold_rad_s_ = declare_parameter<double>(
      "startup_motion_gyro_threshold_rad_s", 0.05);
  startup_motion_accel_delta_threshold_mps2_ = declare_parameter<double>(
      "startup_motion_accel_delta_threshold_mps2", 0.50);
  imu_startup_min_samples_ =
      declare_parameter<int>("imu_startup_min_samples", 200);
  imu_to_cam_translation_ = declare_parameter<std::vector<double>>(
      "imu_to_cam_translation", {0.12, 0.03, 0.242});
  imu_to_cam_rotation_wxyz_ = declare_parameter<std::vector<double>>(
      "imu_to_cam_rotation_wxyz", {0.5, -0.5, 0.5, -0.5});

  if (publish_rate_hz_ <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be > 0");
  }
  if (path_publish_stride_ <= 0) {
    throw std::invalid_argument("path_publish_stride must be > 0");
  }
  if (stereo_sync_queue_size_ <= 0) {
    throw std::invalid_argument("stereo_sync_queue_size must be > 0");
  }
  if (max_pending_imu_inputs_ <= 0 || max_pending_image_inputs_ <= 0) {
    throw std::invalid_argument("pending input queue limits must be > 0");
  }
  if (accepted_input_bag_cache_mb_ < 0) {
    throw std::invalid_argument("accepted_input_bag_cache_mb must be >= 0");
  }
  if (imu_startup_calibration_sec_ < 0.0) {
    throw std::invalid_argument("imu_startup_calibration_sec must be >= 0");
  }
  if (imu_expected_gravity_mps2_ <= 0.0) {
    throw std::invalid_argument("imu_expected_gravity_mps2 must be > 0");
  }
  if (imu_startup_min_samples_ < 0) {
    throw std::invalid_argument("imu_startup_min_samples must be >= 0");
  }
  if (startup_stationary_anchor_std_m_ <= 0.0 ||
      startup_stationary_anchor_rate_hz_ <= 0.0 ||
      startup_motion_gyro_threshold_rad_s_ <= 0.0 ||
      startup_motion_accel_delta_threshold_mps2_ <= 0.0) {
    throw std::invalid_argument("startup stationary-anchor parameters must be > 0");
  }
  if (debug_visual_rate_hz_ <= 0.0) {
    throw std::invalid_argument("debug_visual_rate_hz must be > 0");
  }
  if (max_stereo_input_rate_hz_ < 0.0) {
    throw std::invalid_argument("max_stereo_input_rate_hz must be >= 0");
  }
  if (health_max_accel_bias_mps2_ <= 0.0 ||
      health_max_gyro_bias_rad_s_ <= 0.0 || health_max_speed_mps_ <= 0.0 ||
      health_warning_accel_bias_mps2_ <= 0.0 ||
      health_warning_accel_bias_slope_mps3_ <= 0.0 ||
      health_fault_consecutive_states_ <= 0 ||
      health_warning_consecutive_states_ <= 0 ||
      health_recovery_consecutive_states_ <= 0) {
    throw std::invalid_argument("VIO health thresholds must be > 0");
  }

  opt_flow_out_queue_.set_capacity(50);
  out_state_queue_.set_capacity(50);

  RCLCPP_INFO(get_logger(), "loading calibration");
  if (use_camera_info_calibration_) {
    buildCalibrationFromParameters();
  } else {
    loadCalibration(calib_path_);
  }

  RCLCPP_INFO(get_logger(), "loading config");
  loadConfig(config_path_);

  // Validate the complete input contract before constructing Basalt objects.
  // Basalt starts worker threads from its factories, so throwing afterwards
  // can leave a partially constructed node waiting on those workers.
  expected_cameras_ = calib_.intrinsics.size();
  if (expected_cameras_ != 1 && expected_cameras_ != 2) {
    throw std::runtime_error(
        "basalt_wrapper currently supports only 1 or 2 cameras in the calibration");
  }
  if (expected_cameras_ == 2 && right_image_topic_.empty()) {
    throw std::invalid_argument(
        "right_image_topic must be set for a 2-camera calibration");
  }
  if (input_mode_ != "ros_topics" && input_mode_ != "rosbag2") {
    throw std::invalid_argument("input_mode must be 'ros_topics' or 'rosbag2'");
  }
  if (input_mode_ == "rosbag2" && bag_uri_.empty()) {
    throw std::invalid_argument("bag_uri must be set when input_mode=rosbag2");
  }
  if (!accepted_input_bag_path_.empty() && input_mode_ != "ros_topics") {
    throw std::invalid_argument(
        "accepted_input_bag_path is only valid for input_mode=ros_topics");
  }

  RCLCPP_INFO(get_logger(), "creating optical flow");
  opt_flow_ = basalt::OpticalFlowFactory::getOpticalFlow(vio_config_, calib_);
  if (!opt_flow_) {
    throw std::runtime_error("Basalt optical flow creation returned null");
  }

  RCLCPP_INFO(get_logger(), "creating estimator");
  vio_ = basalt::VioEstimatorFactory::getVioEstimator(
      vio_config_, calib_, basalt::constants::g, use_imu_, use_double_);
  if (!vio_) {
    throw std::runtime_error("Basalt estimator creation returned null");
  }

  RCLCPP_INFO(get_logger(), "initializing estimator");
  vio_->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  RCLCPP_INFO(
      get_logger(),
      "imu startup compensation: enabled=%s window=%.2fs min_samples=%d expected_g=%.4f",
      imu_startup_calibration_enabled_ ? "true" : "false",
      imu_startup_calibration_sec_, imu_startup_min_samples_,
      imu_expected_gravity_mps2_);
  RCLCPP_INFO(
      get_logger(),
      "startup stationary anchor: enabled=%s std=%.3fm rate=%.1fHz release_gyro=%.3frad/s release_accel_delta=%.3fm/s^2",
      startup_stationary_anchor_enabled_ ? "true" : "false",
      startup_stationary_anchor_std_m_, startup_stationary_anchor_rate_hz_,
      startup_motion_gyro_threshold_rad_s_,
      startup_motion_accel_delta_threshold_mps2_);
  RCLCPP_INFO(get_logger(),
              "input liveness limits: pending_imu=%d pending_stereo=%d",
              max_pending_imu_inputs_, max_pending_image_inputs_);
  RCLCPP_INFO(
      get_logger(),
      "estimator health limits: accel_bias=%.3fm/s^2 gyro_bias=%.3frad/s speed=%.1fm/s fault_consecutive=%d recovery_consecutive=%d",
      health_max_accel_bias_mps2_, health_max_gyro_bias_rad_s_,
      health_max_speed_mps_, health_fault_consecutive_states_,
      health_recovery_consecutive_states_);
  RCLCPP_INFO(
      get_logger(),
      "predictive estimator health: accel_bias_warning=%.3fm/s^2 positive_slope_warning=%.3fm/s^3 consecutive=%d",
      health_warning_accel_bias_mps2_,
      health_warning_accel_bias_slope_mps3_,
      health_warning_consecutive_states_);

  RCLCPP_INFO(get_logger(), "wiring queues");
  opt_flow_->output_queue = &opt_flow_out_queue_;
  vio_->out_state_queue = &out_state_queue_;
  if (!marg_data_output_path_.empty()) {
    marg_data_saver_ =
        std::make_shared<basalt::MargDataSaver>(marg_data_output_path_);
    vio_->out_marg_queue = &marg_data_saver_->in_marg_queue;
    RCLCPP_INFO(get_logger(), "recording Basalt marginalization data to %s",
                marg_data_output_path_.c_str());
  }

  // Preserve FIFO order within each physical camera stream while servicing
  // the two eyes concurrently. IMU has a third independent FIFO group, so
  // image transport can never starve inertial delivery.
  vision_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  right_vision_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  imu_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions vision_subscription_options;
  vision_subscription_options.callback_group = vision_callback_group_;
  rclcpp::SubscriptionOptions right_vision_subscription_options;
  right_vision_subscription_options.callback_group =
      right_vision_callback_group_;
  rclcpp::SubscriptionOptions imu_subscription_options;
  imu_subscription_options.callback_group = imu_callback_group_;

  if (expected_cameras_ == 1) {
    if (input_mode_ == "ros_topics") {
      auto image_qos = rclcpp::SensorDataQoS().keep_last(100);
      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
          left_image_topic_, image_qos,
          std::bind(&BasaltNode::imageCallback, this, std::placeholders::_1),
          vision_subscription_options);
      RCLCPP_INFO(get_logger(), "subscribing to mono image topic: %s",
                  left_image_topic_.c_str());
    }
  } else {
    if (input_mode_ == "ros_topics") {
      // The default SensorDataQoS history is only five messages. A short DDS
      // scheduling burst at 1280x800x2 can therefore discard an exposure even
      // though the downstream stereo synchronizer has ample capacity.
      auto image_qos = rclcpp::SensorDataQoS().keep_last(100);
      left_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
          left_image_topic_, image_qos,
          std::bind(&BasaltNode::leftImageCallback, this, std::placeholders::_1),
          vision_subscription_options);
      right_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
          right_image_topic_, image_qos,
          std::bind(&BasaltNode::rightImageCallback, this, std::placeholders::_1),
          right_vision_subscription_options);
      RCLCPP_INFO(get_logger(), "subscribing to stereo image topics: %s %s",
                  left_image_topic_.c_str(), right_image_topic_.c_str());
      RCLCPP_INFO(get_logger(), "stereo estimator rate limit: %s",
                  max_stereo_input_rate_hz_ > 0.0
                      ? (std::to_string(max_stereo_input_rate_hz_) + " Hz").c_str()
                      : "disabled");
    }
  }

  if (use_imu_ && input_mode_ == "ros_topics") {
    auto imu_qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        std::bind(&BasaltNode::imuCallback, this, std::placeholders::_1),
        imu_subscription_options);
    RCLCPP_INFO(get_logger(), "subscribing to IMU topic: %s",
                imu_topic_.c_str());
  }

  if (use_global_position_factors_ && input_mode_ == "ros_topics") {
    global_position_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            global_position_topic_, rclcpp::QoS(20),
            std::bind(&BasaltNode::globalPositionCallback, this,
                      std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "subscribing to global VPC position factors: %s",
                global_position_topic_.c_str());
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/basalt/odometry", rclcpp::QoS(10));
  path_pub_ =
      create_publisher<nav_msgs::msg::Path>("/basalt/path", rclcpp::QoS(10));
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/basalt/pose", rclcpp::QoS(10));
  pose_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud>(
      "/basalt/pose_cloud", rclcpp::QoS(10));
  if (publish_debug_visuals_) {
    auto debug_qos = rclcpp::SensorDataQoS().keep_last(1);
    tracking_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/basalt/tracking_image", debug_qos);
    tracking_overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/basalt/tracking_overlay", debug_qos);
    tracked_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud>(
        "/basalt/tracked_points", debug_qos);
  }
  if (publish_replay_inputs_ && input_mode_ == "ros_topics") {
    auto replay_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    replay_left_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/basalt/replay/cam0/image_raw", replay_qos);
    replay_right_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/basalt/replay/cam1/image_raw", replay_qos);
    replay_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        "/basalt/replay/imu0", replay_qos);
    RCLCPP_INFO(get_logger(),
                "publishing exact accepted replay inputs under /basalt/replay");
  }

  path_msg_.header.frame_id = path_frame_id_;
  pose_cloud_msg_.header.frame_id = path_frame_id_;

  if (!trajectory_output_path_.empty()) {
    trajectory_file_.open(trajectory_output_path_, std::ios::out | std::ios::trunc);
    if (!trajectory_file_.is_open()) {
      throw std::runtime_error("failed to open trajectory_output_path: " +
                               trajectory_output_path_);
    }
    trajectory_file_ << std::unitbuf;
    RCLCPP_INFO(get_logger(), "recording trajectory to %s",
                trajectory_output_path_.c_str());
  }

  if (!ingest_log_path_.empty()) {
    ingest_log_file_.open(ingest_log_path_, std::ios::out | std::ios::trunc);
    if (!ingest_log_file_.is_open()) {
      throw std::runtime_error("failed to open ingest_log_path: " +
                               ingest_log_path_);
    }
    ingest_log_file_ << std::unitbuf;
    ingest_log_file_
        << "push_idx,type,t_ns,source_topic,source_seq,left_hash,right_hash,"
           "imu_count_before_image,imu_hash\n";
    RCLCPP_INFO(get_logger(), "recording ingest log to %s",
                ingest_log_path_.c_str());
  }


  if (!diagnostics_log_path_.empty()) {
    diagnostics_log_file_.open(diagnostics_log_path_,
                               std::ios::out | std::ios::trunc);
    if (!diagnostics_log_file_.is_open()) {
      throw std::runtime_error("failed to open diagnostics_log_path: " +
                               diagnostics_log_path_);
    }
    diagnostics_log_file_ << std::unitbuf;
    diagnostics_log_file_
        << "event,t_ns,wall_ns,value0,value1,value2,value3,value4,value5\n";
    RCLCPP_INFO(get_logger(), "recording VIO diagnostics to %s",
                diagnostics_log_path_.c_str());
  }

  if (!accepted_input_bag_path_.empty()) {
    if (fs::exists(accepted_input_bag_path_)) {
      throw std::runtime_error("accepted_input_bag_path already exists: " +
                               accepted_input_bag_path_);
    }
    const fs::path bag_path(accepted_input_bag_path_);
    if (bag_path.has_parent_path()) {
      fs::create_directories(bag_path.parent_path());
    }
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = accepted_input_bag_path_;
    storage_options.storage_id = "sqlite3";
    storage_options.max_cache_size =
        static_cast<uint64_t>(accepted_input_bag_cache_mb_) * 1024ULL * 1024ULL;
    storage_options.storage_preset_profile = "none";
    accepted_bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
    accepted_bag_writer_->open(storage_options);
    accepted_bag_thread_ =
        std::thread(&BasaltNode::acceptedBagWriterLoop, this);
    RCLCPP_INFO(
        get_logger(),
        "recording exact accepted VIO inputs directly to %s cache=%dMiB",
        accepted_input_bag_path_.c_str(), accepted_input_bag_cache_mb_);
  }

  debug_snapshot_srv_ = create_service<std_srvs::srv::Trigger>(
      "/basalt/get_debug_snapshot",
      std::bind(&BasaltNode::handleDebugSnapshot, this, std::placeholders::_1,
                std::placeholders::_2));

  ingest_thread_ = std::thread(&BasaltNode::ingestLoop, this);
  optical_flow_thread_ =
      std::thread(&BasaltNode::opticalFlowOutputLoop, this);
  estimator_thread_ =
      std::thread(&BasaltNode::estimatorOutputLoop, this);

  if (input_mode_ == "rosbag2") {
    RCLCPP_INFO(get_logger(), "reading input directly from rosbag2: %s",
                bag_uri_.c_str());
    RCLCPP_INFO(get_logger(), "rosbag2 header-time lower bound: %lld ns",
                static_cast<long long>(bag_start_time_ns_));
    bag_input_thread_ = std::thread(&BasaltNode::bagInputLoop, this);
  }

  RCLCPP_INFO(get_logger(), "subscriptions/publisher ready");
}

BasaltNode::~BasaltNode() {
  shutting_down_ = true;

  try {
    ingest_cv_.notify_all();
    latest_imu_cv_.notify_all();
    if (bag_input_thread_.joinable()) {
      bag_input_thread_.join();
    }
    if (ingest_thread_.joinable()) {
      ingest_thread_.join();
    }
    if (accepted_bag_writer_) {
      accepted_bag_stop_ = true;
      accepted_bag_cv_.notify_all();
      if (accepted_bag_thread_.joinable()) {
        accepted_bag_thread_.join();
      }
      accepted_bag_writer_->close();
      RCLCPP_INFO(
          get_logger(),
          "closed accepted-input bag: imu=%llu images=%llu stereo_pairs=%llu failed=%s",
          static_cast<unsigned long long>(accepted_bag_imu_written_),
          static_cast<unsigned long long>(accepted_bag_images_written_),
          static_cast<unsigned long long>(accepted_bag_images_written_ / 2),
          accepted_bag_failed_.load() ? "true" : "false");
      accepted_bag_writer_.reset();
    }
    if (opt_flow_) {
      opt_flow_->input_queue.push(nullptr);
    }
    if (vio_) {
      if (use_imu_) {
        // Send this before the visual sentinel. Once Basalt consumes a null
        // visual frame it stops reading IMU, so the reverse order can block
        // forever when a live-topic IMU queue is full at shutdown.
        vio_->imu_data_queue.push(nullptr);
      }
      // The wrapper owns the optical-flow forwarding thread. Once shutdown
      // starts it drains rather than forwards, so terminate Basalt's visual
      // queue explicitly instead of waiting forever for a forwarded sentinel.
      vio_->vision_data_queue.push(nullptr);
      vio_->maybe_join();
      vio_->drain_input_queues();
    }
    marg_data_saver_.reset();
    if (optical_flow_thread_.joinable()) {
      optical_flow_thread_.join();
    }
    if (estimator_thread_.joinable()) {
      estimator_thread_.join();
    }
    basalt::PoseVelBiasState<double>::Ptr state;
    while (out_state_queue_.try_pop(state)) {
    }
    if (trajectory_file_.is_open()) {
      trajectory_file_.flush();
      trajectory_file_.close();
    }
    if (ingest_log_file_.is_open()) {
      ingest_log_file_.flush();
      ingest_log_file_.close();
    }
    if (diagnostics_log_file_.is_open()) {
      diagnostics_log_file_.flush();
      diagnostics_log_file_.close();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "shutdown error: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "shutdown error: unknown exception");
  }
}

int64_t BasaltNode::rosTimeToNs(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

int64_t BasaltNode::steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int64_t BasaltNode::imageTimestampNs(
    const builtin_interfaces::msg::Time &stamp) const {
  if (use_header_timestamps_) {
    return rosTimeToNs(stamp);
  }
  return steadyNowNs();
}

uint64_t BasaltNode::hashImageMessage(const ImageConstPtr &msg) const {
  if (!ingest_log_file_.is_open()) {
    return 0;
  }
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  SHA256(msg->data.data(), msg->data.size(), digest.data());
  uint64_t hash = 0;
  std::memcpy(&hash, digest.data(), sizeof(hash));
  return hash;
}

uint64_t BasaltNode::hashImuMessage(
    const sensor_msgs::msg::Imu::SharedPtr &msg) const {
  constexpr uint64_t kOffset = 1469598103934665603ULL;
  constexpr uint64_t kPrime = 1099511628211ULL;
  uint64_t hash = kOffset;
  const std::array<double, 6> consumed_values{
      msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z, msg->linear_acceleration.x,
      msg->linear_acceleration.y, msg->linear_acceleration.z};
  for (const double value : consumed_values) {
    const auto *bytes = reinterpret_cast<const uint8_t *>(&value);
    for (size_t index = 0; index < sizeof(value); ++index) {
      hash ^= static_cast<uint64_t>(bytes[index]);
      hash *= kPrime;
    }
  }
  return hash;
}

bool BasaltNode::acceptImageTimestampNs(int64_t candidate_ns,
                                        int64_t &accepted_ns) {
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  if (candidate_ns < last_image_t_ns_) {
    return false;
  }
  if (candidate_ns == last_image_t_ns_) {
    // Some simulator/bridge combinations can emit distinct consecutive stereo
    // frames with the same header stamp. Keep the sequence strictly monotonic
    // so Basalt continues ingesting instead of stalling on duplicates.
    accepted_ns = last_image_t_ns_ + 1;
    last_image_t_ns_ = accepted_ns;
    return true;
  }
  last_image_t_ns_ = candidate_ns;
  accepted_ns = candidate_ns;
  return true;
}

void BasaltNode::enqueueImageInput(PendingImageInput input) {
  bool liveness_fault = false;
  size_t pending_size = 0;
  int64_t oldest_t_ns = 0;
  int64_t newest_t_ns = 0;
  {
    std::lock_guard<std::mutex> lock(ingest_mutex_);
    if (input_liveness_faulted_) {
      return;
    }
    auto it = std::upper_bound(
        pending_image_inputs_.begin(), pending_image_inputs_.end(), input.t_ns,
        [](int64_t t_ns, const PendingImageInput &candidate) {
          return t_ns < candidate.t_ns;
        });
    pending_image_inputs_.insert(it, std::move(input));
    pending_size = pending_image_inputs_.size();
    oldest_t_ns = pending_image_inputs_.front().t_ns;
    newest_t_ns = pending_image_inputs_.back().t_ns;
    liveness_fault = pending_size > static_cast<size_t>(max_pending_image_inputs_);
    if (liveness_fault) {
      input_liveness_faulted_ = true;
    }
    const bool should_log =
        ((pending_size <= 5 && pending_size != last_logged_pending_image_queue_size_) ||
         (pending_size >= 500 && pending_size % 500 == 0 &&
          pending_size != last_logged_pending_image_queue_size_));
    if (should_log) {
      last_logged_pending_image_queue_size_ = pending_size;
      RCLCPP_INFO(get_logger(),
                  "pending stereo queue size=%zu oldest_t_ns=%lld newest_t_ns=%lld",
                  pending_size, static_cast<long long>(oldest_t_ns),
                  static_cast<long long>(newest_t_ns));
    }
    ingest_cv_.notify_one();
  }
  if (liveness_fault) {
    RCLCPP_FATAL(
        get_logger(),
        "Basalt input liveness failure: pending stereo queue exceeded limit "
        "size=%zu limit=%d oldest_t_ns=%lld newest_t_ns=%lld. Stopping instead "
        "of publishing a frozen pose.",
        pending_size, max_pending_image_inputs_,
        static_cast<long long>(oldest_t_ns),
        static_cast<long long>(newest_t_ns));
    shutting_down_ = true;
    ingest_cv_.notify_all();
    latest_imu_cv_.notify_all();
    rclcpp::shutdown();
  }
}

void BasaltNode::enqueueImuInput(PendingImuInput input) {
  bool liveness_fault = false;
  size_t pending_size = 0;
  int64_t oldest_t_ns = 0;
  int64_t newest_t_ns = 0;
  {
    std::lock_guard<std::mutex> lock(ingest_mutex_);
    if (input_liveness_faulted_) {
      return;
    }
    auto it = std::upper_bound(
        pending_imu_inputs_.begin(), pending_imu_inputs_.end(), input.t_ns,
        [](int64_t t_ns, const PendingImuInput &candidate) {
          return t_ns < candidate.t_ns;
        });
    pending_imu_inputs_.insert(it, std::move(input));
    pending_size = pending_imu_inputs_.size();
    oldest_t_ns = pending_imu_inputs_.front().t_ns;
    newest_t_ns = pending_imu_inputs_.back().t_ns;
    liveness_fault = pending_size > static_cast<size_t>(max_pending_imu_inputs_);
    if (liveness_fault) {
      input_liveness_faulted_ = true;
    }
    const bool should_log =
        ((pending_size <= 5 && pending_size != last_logged_pending_imu_queue_size_) ||
         (pending_size >= 1000 && pending_size % 1000 == 0 &&
          pending_size != last_logged_pending_imu_queue_size_));
    if (should_log) {
      last_logged_pending_imu_queue_size_ = pending_size;
      RCLCPP_INFO(get_logger(),
                  "pending imu queue size=%zu oldest_t_ns=%lld newest_t_ns=%lld",
                  pending_size, static_cast<long long>(oldest_t_ns),
                  static_cast<long long>(newest_t_ns));
    }
    ingest_cv_.notify_one();
  }
  if (liveness_fault) {
    RCLCPP_FATAL(
        get_logger(),
        "Basalt input liveness failure: pending IMU queue exceeded limit "
        "size=%zu limit=%d oldest_t_ns=%lld newest_t_ns=%lld. Stopping instead "
        "of publishing a frozen pose.",
        pending_size, max_pending_imu_inputs_,
        static_cast<long long>(oldest_t_ns),
        static_cast<long long>(newest_t_ns));
    shutting_down_ = true;
    ingest_cv_.notify_all();
    latest_imu_cv_.notify_all();
    rclcpp::shutdown();
  }
}

bool BasaltNode::dropStaleUnmatchedStereoLocked(int64_t *dropped_t_ns) {
  // Do not pair adjacent 60 Hz exposures when one camera frame is missing.
  constexpr int64_t kStereoMatchToleranceNs = 1 * 1000 * 1000;  // 1 ms

  const bool have_left = !pending_left_images_.empty();
  const bool have_right = !pending_right_images_.empty();
  if (!have_left || !have_right) {
    return false;
  }

  const int64_t left_t = pending_left_images_.begin()->first;
  const int64_t right_t = pending_right_images_.begin()->first;

  if (std::llabs(left_t - right_t) <= kStereoMatchToleranceNs) {
    return false;
  }

  if (left_t < right_t) {
    if (dropped_t_ns) {
      *dropped_t_ns = left_t;
    }
    pending_left_images_.erase(pending_left_images_.begin());
    return true;
  }

  if (dropped_t_ns) {
    *dropped_t_ns = right_t;
  }
  pending_right_images_.erase(pending_right_images_.begin());
  return true;
}

void BasaltNode::ingestLoop() {
  while (!shutting_down_) {
    PendingImageInput image_input;
    PendingImuInput imu_input;
    bool process_image = false;
    bool process_imu = false;

    {
      std::unique_lock<std::mutex> lock(ingest_mutex_);
      ingest_cv_.wait(lock, [&] {
        return shutting_down_ || !pending_image_inputs_.empty() ||
               !pending_imu_inputs_.empty();
      });

      if (shutting_down_) {
        break;
      }

      const bool have_image = !pending_image_inputs_.empty();
      const bool have_imu = !pending_imu_inputs_.empty();
      if (have_image &&
          (!have_imu ||
           pending_image_inputs_.front().t_ns <= pending_imu_inputs_.front().t_ns)) {
        image_input = std::move(pending_image_inputs_.front());
        pending_image_inputs_.pop_front();
        process_image = true;
      } else if (have_imu) {
        // Do not let a single unmatched left/right camera component block the
        // entire VIO ingestion stream. ros_gz_image can occasionally deliver
        // a duplicate or lose one side of a stereo frame. The old ordering
        // gate then stopped both IMU and already-paired stereo ingestion
        // forever, causing all queues to grow and PX4 external vision to die.
        // Basalt has independent timestamped IMU and vision queues, so it is
        // safe to continue feeding IMU while the stereo matcher waits for or
        // discards the orphaned component.
        imu_input = std::move(pending_imu_inputs_.front());
        pending_imu_inputs_.pop_front();
        process_imu = true;
      } else {
        int64_t dropped_t_ns = 0;
        if (dropStaleUnmatchedStereoLocked(&dropped_t_ns)) {
          ++unmatched_stereo_dropped_;
          if (unmatched_stereo_dropped_ <= 5 ||
              unmatched_stereo_dropped_ % 500 == 0) {
            RCLCPP_WARN(
                get_logger(),
                "dropping stale unmatched stereo component at t_ns=%lld dropped=%zu",
                static_cast<long long>(dropped_t_ns),
                unmatched_stereo_dropped_);
          }
          continue;
        }
        ingest_cv_.wait(lock);
        continue;
      }
    }

    if (process_imu) {
      auto imu = std::make_shared<basalt::ImuData<double>>();
      if (!acceptImuTimestampNs(imu_input.t_ns, imu->t_ns)) {
        ++imu_dropped_out_of_order_;
        if (imu_dropped_out_of_order_ <= 5 ||
            imu_dropped_out_of_order_ % 1000 == 0) {
          RCLCPP_WARN(
              get_logger(),
              "dropping out-of-order imu sample stamp=%lld last_imu_t_ns=%lld dropped=%zu",
              static_cast<long long>(imu_input.t_ns),
              static_cast<long long>(last_imu_input_t_ns_),
              imu_dropped_out_of_order_);
        }
        continue;
      }

      const auto &msg = imu_input.msg;
      if (replay_imu_pub_) {
        replay_imu_pub_->publish(*msg);
      }
      const Eigen::Vector3d raw_gyro(msg->angular_velocity.x,
                                     msg->angular_velocity.y,
                                     msg->angular_velocity.z);
      const Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                                      msg->linear_acceleration.y,
                                      msg->linear_acceleration.z);
      imu->gyro = raw_gyro;
      imu->accel = raw_accel;
      applyStartupImuCompensation(imu->gyro, imu->accel, imu->t_ns);

      // Do not fill Basalt's bounded IMU queue while visual initialization is
      // intentionally gated. Basalt cannot consume those samples until its
      // first image, so a multi-second stationary window otherwise deadlocks
      // before it can ever become ready.
      const bool forward_to_vio =
          !imu_startup_calibration_enabled_ || imu_startup_bias_ready_;
      if (forward_to_vio) {
        vio_->imu_data_queue.push(imu);
        ++imu_pushes_since_last_image_;
      }
      storeLatestImuDebug(raw_gyro, raw_accel, imu->gyro, imu->accel, imu->t_ns);
      // The replay dataset must include the stationary samples consumed by
      // startup calibration as well as samples forwarded to Basalt afterward.
      // Otherwise replay starts after the calibration window and cannot
      // reconstruct the same initialized estimator input.
      logIngestPushImu(++next_ingest_push_idx_, imu_input);
      enqueueAcceptedBagImu(imu_input);
      ++accepted_ingest_events_completed_;
      latest_imu_cv_.notify_all();
      ++imu_received_;
      if (imu_received_ <= 5 || imu_received_ % 500 == 0) {
        RCLCPP_INFO(
            get_logger(),
            "ingested imu #%zu t_ns=%lld queue_remaining=%zu latest_image_t_ns=%lld",
            imu_received_, static_cast<long long>(imu->t_ns),
            pending_imu_inputs_.size(),
            static_cast<long long>(last_image_t_ns_));
      }
      continue;
    }

    if (process_image) {
      if (imu_startup_calibration_enabled_ && !imu_startup_bias_ready_) {
        ++startup_images_dropped_;
        if (startup_images_dropped_ <= 3 ||
            startup_images_dropped_ % 20 == 0) {
          RCLCPP_INFO(
              get_logger(),
              "holding visual initialization until stationary IMU window is ready; dropped_startup_images=%zu",
              startup_images_dropped_);
        }
        continue;
      }
      // Camera callbacks only pair and rate-select shared message pointers.
      // Defer full-frame integrity hashes to this single ingest worker so the
      // callback lock remains available for both 60 Hz bridge streams.
      if (image_input.msgs.size() == 2 && image_input.left_hash == 0 &&
          image_input.right_hash == 0) {
        image_input.left_hash = hashImageMessage(image_input.msgs[0]);
        image_input.right_hash = hashImageMessage(image_input.msgs[1]);
      }
      if (replay_left_image_pub_ && replay_right_image_pub_ &&
          image_input.msgs.size() == 2) {
        replay_left_image_pub_->publish(*image_input.msgs[0]);
        replay_right_image_pub_->publish(*image_input.msgs[1]);
      }
      std_msgs::msg::Header debug_header;
      auto input =
          makeOpticalFlowInput(image_input.msgs, image_input.t_ns, debug_header);
      if (!input) {
        continue;
      }

      const int64_t image_minus_imu_ns =
          latest_imu_t_ns_ == 0 ? std::numeric_limits<int64_t>::min()
                                : input->t_ns - latest_imu_t_ns_;
      if (!validateCrossStreamTiming(input->t_ns, latest_imu_t_ns_)) {
        break;
      }
      if (images_received_ < 5 || images_received_ % 500 == 0) {
        if (latest_imu_t_ns_ == 0) {
          RCLCPP_WARN(
              get_logger(),
              "ingesting stereo t_ns=%lld with no IMU received yet, stereo_match_delta_ns=%lld pending_imu=%zu pending_stereo=%zu",
              static_cast<long long>(input->t_ns),
              static_cast<long long>(last_stereo_match_delta_ns_),
              pending_imu_inputs_.size(), pending_image_inputs_.size());
        } else {
          RCLCPP_INFO(
              get_logger(),
              "ingesting stereo #%zu t_ns=%lld latest_imu_t_ns=%lld image_minus_imu_ns=%lld stereo_match_delta_ns=%lld pending_imu=%zu pending_stereo=%zu",
              images_received_ + 1, static_cast<long long>(input->t_ns),
              static_cast<long long>(latest_imu_t_ns_),
              static_cast<long long>(image_minus_imu_ns),
              static_cast<long long>(last_stereo_match_delta_ns_),
              pending_imu_inputs_.size(), pending_image_inputs_.size());
        }
      }
      opt_flow_->input_queue.push(input);
      maybeAddStartupStationaryAnchor();
      logIngestPushStereo(++next_ingest_push_idx_, image_input,
                          imu_pushes_since_last_image_);
      enqueueAcceptedBagStereo(image_input);
      ++accepted_ingest_events_completed_;
      imu_pushes_since_last_image_ = 0;
      ++images_received_;
      if (images_received_ % 500 == 0) {
        RCLCPP_INFO(get_logger(),
                    "%s frames received=%zu last_image_t_ns=%lld last_imu_t_ns=%lld",
                    expected_cameras_ == 2 ? "stereo" : "image",
                    images_received_, static_cast<long long>(input->t_ns),
                    static_cast<long long>(latest_imu_t_ns_));
      }
    }
  }
}

void BasaltNode::bagInputLoop() {
  try {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_uri_;
    storage_options.storage_id = bag_storage_id_;

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer;
    rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped>
        global_position_serializer;

    struct BagEvent {
      int64_t t_ns{0};
      int priority{0};
      enum class Kind { MonoImage, LeftImage, RightImage, Imu, GlobalPosition } kind{
          Kind::Imu};
      sensor_msgs::msg::Image image;
      sensor_msgs::msg::Imu imu;
      geometry_msgs::msg::PoseWithCovarianceStamped global_position;
    };

    std::vector<BagEvent> events;
    int64_t bag_target_image_t_ns = 0;

    while (!shutting_down_ && reader.has_next()) {
      auto bag_msg = reader.read_next();
      if (!bag_msg || !bag_msg->serialized_data) {
        continue;
      }

      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      if (expected_cameras_ == 1 && bag_msg->topic_name == left_image_topic_) {
        sensor_msgs::msg::Image msg;
        image_serializer.deserialize_message(&serialized_msg, &msg);
        BagEvent event;
        event.t_ns = rosTimeToNs(msg.header.stamp);
        if (bag_start_time_ns_ > 0 && event.t_ns < bag_start_time_ns_) continue;
        event.priority = 1;
        event.kind = BagEvent::Kind::MonoImage;
        event.image = std::move(msg);
        bag_target_image_t_ns = std::max(bag_target_image_t_ns, event.t_ns);
        events.push_back(std::move(event));
      } else if (bag_msg->topic_name == left_image_topic_) {
        sensor_msgs::msg::Image msg;
        image_serializer.deserialize_message(&serialized_msg, &msg);
        BagEvent event;
        event.t_ns = rosTimeToNs(msg.header.stamp);
        if (bag_start_time_ns_ > 0 && event.t_ns < bag_start_time_ns_) continue;
        event.priority = 1;
        event.kind = BagEvent::Kind::LeftImage;
        event.image = std::move(msg);
        bag_target_image_t_ns = std::max(bag_target_image_t_ns, event.t_ns);
        events.push_back(std::move(event));
      } else if (bag_msg->topic_name == right_image_topic_) {
        sensor_msgs::msg::Image msg;
        image_serializer.deserialize_message(&serialized_msg, &msg);
        BagEvent event;
        event.t_ns = rosTimeToNs(msg.header.stamp);
        if (bag_start_time_ns_ > 0 && event.t_ns < bag_start_time_ns_) continue;
        event.priority = 2;
        event.kind = BagEvent::Kind::RightImage;
        event.image = std::move(msg);
        bag_target_image_t_ns = std::max(bag_target_image_t_ns, event.t_ns);
        events.push_back(std::move(event));
      } else if (use_imu_ && bag_msg->topic_name == imu_topic_) {
        sensor_msgs::msg::Imu msg;
        imu_serializer.deserialize_message(&serialized_msg, &msg);
        BagEvent event;
        event.t_ns = rosTimeToNs(msg.header.stamp);
        if (bag_start_time_ns_ > 0 && event.t_ns < bag_start_time_ns_) continue;
        event.priority = 0;
        event.kind = BagEvent::Kind::Imu;
        event.imu = std::move(msg);
        events.push_back(std::move(event));
      } else if (use_global_position_factors_ &&
                 bag_msg->topic_name == global_position_topic_) {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        global_position_serializer.deserialize_message(&serialized_msg, &msg);
        BagEvent event;
        event.t_ns = rosTimeToNs(msg.header.stamp);
        if (bag_start_time_ns_ > 0 && event.t_ns < bag_start_time_ns_) continue;
        event.priority = 3;
        event.kind = BagEvent::Kind::GlobalPosition;
        event.global_position = std::move(msg);
        events.push_back(std::move(event));
      }
    }

    if (!bag_preserve_record_order_) {
      std::stable_sort(events.begin(), events.end(),
                       [](const BagEvent &a, const BagEvent &b) {
                         if (a.t_ns != b.t_ns) {
                           return a.t_ns < b.t_ns;
                         }
                         return a.priority < b.priority;
                       });
    }

    RCLCPP_INFO(
        get_logger(), "feeding %zu rosbag2 sensor events in %s order",
        events.size(),
        bag_preserve_record_order_ ? "recorded accepted-event" : "header-time");

    auto waitForOfflineBackpressure = [&]() {
      using namespace std::chrono_literals;
      auto last_log = std::chrono::steady_clock::now();
      while (!shutting_down_) {
        size_t pending_images = 0;
        size_t pending_imu = 0;
        size_t pending_left = 0;
        size_t pending_right = 0;
        {
          std::lock_guard<std::mutex> lock(ingest_mutex_);
          pending_images = pending_image_inputs_.size();
          pending_imu = pending_imu_inputs_.size();
          pending_left = pending_left_images_.size();
          pending_right = pending_right_images_.size();
        }

        if (pending_images < 25 && pending_imu < 750 && pending_left < 25 &&
            pending_right < 25) {
          return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_log > 2s) {
          last_log = now;
          RCLCPP_INFO(
              get_logger(),
              "rosbag2 offline feed waiting for estimator: pending_stereo=%zu pending_imu=%zu pending_left=%zu pending_right=%zu",
              pending_images, pending_imu, pending_left, pending_right);
        }
        std::this_thread::sleep_for(5ms);
      }
    };

    auto waitForAcceptedIngest = [&](uint64_t target) {
      using namespace std::chrono_literals;
      const auto deadline = std::chrono::steady_clock::now() + 30s;
      while (!shutting_down_ &&
             accepted_ingest_events_completed_.load() < target) {
        if (std::chrono::steady_clock::now() >= deadline) {
          throw std::runtime_error(
              "timed out preserving recorded accepted-event order");
        }
        std::this_thread::sleep_for(1ms);
      }
    };

    for (auto &event : events) {
      if (shutting_down_) {
        break;
      }
      const uint64_t completed_before =
          accepted_ingest_events_completed_.load();
      bool wait_for_accepted_ingest = false;
      switch (event.kind) {
        case BagEvent::Kind::MonoImage:
          imageCallback(
              std::make_shared<sensor_msgs::msg::Image>(std::move(event.image)));
          wait_for_accepted_ingest = true;
          break;
        case BagEvent::Kind::LeftImage:
          leftImageCallback(
              std::make_shared<sensor_msgs::msg::Image>(std::move(event.image)));
          break;
        case BagEvent::Kind::RightImage:
          rightImageCallback(
              std::make_shared<sensor_msgs::msg::Image>(std::move(event.image)));
          wait_for_accepted_ingest = true;
          break;
        case BagEvent::Kind::Imu:
          imuCallback(
              std::make_shared<sensor_msgs::msg::Imu>(std::move(event.imu)));
          wait_for_accepted_ingest = true;
          break;
        case BagEvent::Kind::GlobalPosition:
          globalPositionCallback(
              std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(
                  std::move(event.global_position)));
          break;
      }
      if (bag_preserve_record_order_ && wait_for_accepted_ingest) {
        waitForAcceptedIngest(completed_before + 1);
      } else {
        waitForOfflineBackpressure();
      }
    }

    RCLCPP_INFO(get_logger(), "finished reading rosbag2 input: %s",
                bag_uri_.c_str());

    const int64_t target_image_t_ns =
        bag_target_image_t_ns > 0 ? bag_target_image_t_ns : last_image_t_ns_;
    if (target_image_t_ns > 0 && !shutting_down_) {
      using namespace std::chrono_literals;
      const auto drain_deadline = std::chrono::steady_clock::now() + 5min;
      int64_t last_progress_state_t_ns = latest_published_state_t_ns_.load();
      auto last_progress_tp = std::chrono::steady_clock::now();

      while (!shutting_down_ && std::chrono::steady_clock::now() < drain_deadline) {
        const auto now = std::chrono::steady_clock::now();
        const int64_t latest_state_t_ns = latest_published_state_t_ns_.load();
        if (latest_state_t_ns > last_progress_state_t_ns) {
          last_progress_state_t_ns = latest_state_t_ns;
          last_progress_tp = now;
        }

        bool ingest_drained = false;
        {
          std::lock_guard<std::mutex> lock(ingest_mutex_);
          const bool no_pending_images = pending_image_inputs_.empty() &&
                                         pending_left_images_.empty() &&
                                         pending_right_images_.empty();
          const bool only_trailing_imu =
              pending_imu_inputs_.empty() ||
              pending_imu_inputs_.front().t_ns > target_image_t_ns;
          ingest_drained = no_pending_images && only_trailing_imu;
        }

        const bool reached_target = latest_state_t_ns >= target_image_t_ns;
        const bool no_recent_progress =
            last_progress_state_t_ns > 0 && (now - last_progress_tp) > 5s;

        if (ingest_drained && (reached_target || no_recent_progress)) {
          RCLCPP_INFO(
              get_logger(),
              "rosbag2 drain complete: target_image_t_ns=%lld latest_state_t_ns=%lld ingest_drained=%s stalled=%s",
              static_cast<long long>(target_image_t_ns),
              static_cast<long long>(latest_state_t_ns),
              ingest_drained ? "true" : "false",
              no_recent_progress ? "true" : "false");
          break;
        }

        std::this_thread::sleep_for(50ms);
      }

      if (!shutting_down_ &&
          std::chrono::steady_clock::now() >= drain_deadline) {
        int64_t oldest_pending_image_t_ns = -1;
        int64_t oldest_pending_imu_t_ns = -1;
        {
          std::lock_guard<std::mutex> lock(ingest_mutex_);
          if (!pending_image_inputs_.empty()) {
            oldest_pending_image_t_ns = pending_image_inputs_.front().t_ns;
          }
          if (!pending_imu_inputs_.empty()) {
            oldest_pending_imu_t_ns = pending_imu_inputs_.front().t_ns;
          }
        }
        RCLCPP_WARN(
            get_logger(),
            "rosbag2 drain wait timed out: target_image_t_ns=%lld latest_state_t_ns=%lld pending_image_t_ns=%lld pending_imu_t_ns=%lld",
            static_cast<long long>(target_image_t_ns),
            static_cast<long long>(latest_published_state_t_ns_.load()),
            static_cast<long long>(oldest_pending_image_t_ns),
            static_cast<long long>(oldest_pending_imu_t_ns));
      }
    }

    if (!shutting_down_) {
      RCLCPP_INFO(get_logger(),
                  "shutting down after rosbag2 playback completion: %s",
                  bag_uri_.c_str());
      shutting_down_ = true;
      ingest_cv_.notify_all();
      latest_imu_cv_.notify_all();
      rclcpp::shutdown();
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "rosbag2 input loop failed: %s", e.what());
  }
}

void BasaltNode::logIngestPushImu(uint64_t push_idx,
                                  const PendingImuInput &imu_input) {
  if (!ingest_log_file_.is_open()) {
    return;
  }
  std::lock_guard<std::mutex> lock(ingest_log_mutex_);
  ingest_log_file_ << push_idx << ",imu," << imu_input.t_ns << ","
                   << imu_input.source_topic << "," << imu_input.seq
                   << ",0,0,0," << hashImuMessage(imu_input.msg) << "\n";
}

void BasaltNode::logIngestPushStereo(uint64_t push_idx,
                                     const PendingImageInput &image_input,
                                     size_t imu_count_before_image) {
  if (!ingest_log_file_.is_open()) {
    return;
  }
  std::lock_guard<std::mutex> lock(ingest_log_mutex_);
  ingest_log_file_ << push_idx << ",stereo," << image_input.t_ns << ","
                   << image_input.source_topic << "," << image_input.seq << ","
                   << image_input.left_hash << "," << image_input.right_hash
                   << "," << imu_count_before_image << ",0\n";
}

void BasaltNode::enqueueAcceptedBagImu(const PendingImuInput &imu_input) {
  if (!accepted_bag_writer_) {
    return;
  }
  AcceptedBagWrite item;
  item.t_ns = imu_input.t_ns;
  item.topic = "/basalt/replay/imu0";
  item.imu = imu_input.msg;
  {
    std::lock_guard<std::mutex> lock(accepted_bag_mutex_);
    item.record_order = ++accepted_bag_next_record_order_;
    accepted_bag_queue_.push_back(std::move(item));
  }
  accepted_bag_cv_.notify_one();
}

void BasaltNode::enqueueAcceptedBagStereo(
    const PendingImageInput &image_input) {
  if (!accepted_bag_writer_) {
    return;
  }
  if (image_input.msgs.size() != 2) {
    throw std::runtime_error(
        "accepted-input stereo recording requires exactly two images");
  }
  AcceptedBagWrite left;
  left.t_ns = image_input.t_ns;
  left.topic = "/basalt/replay/cam0/image_raw";
  left.image = image_input.msgs[0];
  AcceptedBagWrite right;
  right.t_ns = image_input.t_ns;
  right.topic = "/basalt/replay/cam1/image_raw";
  right.image = image_input.msgs[1];
  {
    std::lock_guard<std::mutex> lock(accepted_bag_mutex_);
    left.record_order = ++accepted_bag_next_record_order_;
    right.record_order = ++accepted_bag_next_record_order_;
    accepted_bag_queue_.push_back(std::move(left));
    accepted_bag_queue_.push_back(std::move(right));
  }
  accepted_bag_cv_.notify_one();
}

void BasaltNode::acceptedBagWriterLoop() {
  try {
    while (true) {
      AcceptedBagWrite item;
      {
        std::unique_lock<std::mutex> lock(accepted_bag_mutex_);
        accepted_bag_cv_.wait(lock, [&] {
          return accepted_bag_stop_.load() || !accepted_bag_queue_.empty();
        });
        if (accepted_bag_queue_.empty()) {
          if (accepted_bag_stop_.load()) {
            break;
          }
          continue;
        }
        item = std::move(accepted_bag_queue_.front());
        accepted_bag_queue_.pop_front();
      }

      if (item.image) {
        accepted_bag_writer_->write(*item.image, item.topic,
                                    rclcpp::Time(item.record_order));
        ++accepted_bag_images_written_;
      } else if (item.imu) {
        accepted_bag_writer_->write(*item.imu, item.topic,
                                    rclcpp::Time(item.record_order));
        ++accepted_bag_imu_written_;
      } else {
        throw std::runtime_error("accepted-input bag item has no message");
      }
    }
  } catch (const std::exception &e) {
    accepted_bag_failed_ = true;
    RCLCPP_FATAL(get_logger(), "accepted-input bag writer failed: %s", e.what());
    shutting_down_ = true;
    ingest_cv_.notify_all();
    latest_imu_cv_.notify_all();
    rclcpp::shutdown();
  }
}

bool BasaltNode::acceptImuTimestampNs(int64_t candidate_ns,
                                      int64_t &accepted_ns) {
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  if (candidate_ns <= last_imu_input_t_ns_) {
    return false;
  }
  last_imu_input_t_ns_ = candidate_ns;
  accepted_ns = candidate_ns;
  return true;
}

bool BasaltNode::validateCrossStreamTiming(int64_t image_t_ns, int64_t imu_t_ns) {
  if (imu_t_ns == 0) {
    return true;
  }

  constexpr int64_t kMaxAcceptedImageImuDeltaNs = 5LL * 1000 * 1000 * 1000;
  const int64_t delta_ns = image_t_ns - imu_t_ns;
  if (std::llabs(delta_ns) <= kMaxAcceptedImageImuDeltaNs) {
    return true;
  }

  if (!cross_stream_timing_error_logged_) {
    cross_stream_timing_error_logged_ = true;
    RCLCPP_FATAL(
        get_logger(),
        "image/imu timestamps are in different clock domains: image_t_ns=%lld "
        "latest_imu_t_ns=%lld delta_ns=%lld. Basalt requires a shared time "
        "base across all sensor streams. For the PX4/Gazebo stack, launch "
        "px4_basalt_bridge with timestamp_mode:=ros and use_sim_time:=true.",
        static_cast<long long>(image_t_ns), static_cast<long long>(imu_t_ns),
        static_cast<long long>(delta_ns));
  }

  shutting_down_ = true;
  ingest_cv_.notify_all();
  latest_imu_cv_.notify_all();
  rclcpp::shutdown();
  return false;
}

std::array<double, 36> BasaltNode::defaultCovariance(double linear,
                                                     double angular) {
  std::array<double, 36> cov{};
  cov.fill(0.0);
  cov[0] = linear;
  cov[7] = linear;
  cov[14] = linear;
  cov[21] = angular;
  cov[28] = angular;
  cov[35] = angular;
  return cov;
}

std::string BasaltNode::formatVector(const Eigen::Vector3d &v) const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(6);
  oss << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  return oss.str();
}

std::string BasaltNode::formatQuat(const Eigen::Quaterniond &q) const {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(6);
  oss << "[w=" << q.w() << ", x=" << q.x() << ", y=" << q.y()
      << ", z=" << q.z() << "]";
  return oss.str();
}

void BasaltNode::requireExistingFile(const std::string &path,
                                     const char *label) const {
  if (path.empty()) {
    RCLCPP_FATAL(get_logger(), "%s parameter is empty", label);
    throw std::invalid_argument(std::string(label) + " parameter is empty");
  }

  if (!fs::exists(path)) {
    RCLCPP_FATAL(get_logger(), "%s does not exist: %s", label, path.c_str());
    throw std::runtime_error(std::string(label) + " does not exist: " + path);
  }

  if (!fs::is_regular_file(path)) {
    RCLCPP_FATAL(get_logger(), "%s is not a file: %s", label, path.c_str());
    throw std::runtime_error(std::string(label) + " is not a file: " + path);
  }
}

void BasaltNode::loadCalibration(const std::string &path) {
  requireExistingFile(path, "calib_path");

  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    RCLCPP_FATAL(get_logger(), "failed to open calibration file: %s",
                 path.c_str());
    throw std::runtime_error("failed to open calibration file: " + path);
  }

  cereal::JSONInputArchive archive(input);
  archive(calib_);

  RCLCPP_INFO(get_logger(), "loaded calibration from %s with %zu cameras",
              path.c_str(), calib_.intrinsics.size());

  if (calib_.intrinsics.empty() ||
      calib_.intrinsics.size() != calib_.T_i_c.size() ||
      calib_.intrinsics.size() != calib_.resolution.size()) {
    RCLCPP_FATAL(get_logger(),
                 "invalid calibration: intrinsics=%zu T_i_c=%zu resolution=%zu",
                 calib_.intrinsics.size(), calib_.T_i_c.size(),
                 calib_.resolution.size());
    throw std::runtime_error("invalid calibration layout");
  }

  auto require_positive_vec3 = [&](const Eigen::Vector3d &value,
                                   const char *label) {
    if ((value.array() <= 0.0).any()) {
      RCLCPP_FATAL(get_logger(),
                   "%s must be strictly positive for Basalt VIO, got [%.9f %.9f %.9f]",
                   label, value.x(), value.y(), value.z());
      throw std::runtime_error(std::string("invalid calibration: non-positive ") +
                               label);
    }
  };

  if (use_imu_) {
    require_positive_vec3(calib_.accel_noise_std, "accel_noise_std");
    require_positive_vec3(calib_.gyro_noise_std, "gyro_noise_std");
    require_positive_vec3(calib_.accel_bias_std, "accel_bias_std");
    require_positive_vec3(calib_.gyro_bias_std, "gyro_bias_std");
  }

  const auto &t_i_c = calib_.T_i_c.front();
  const Eigen::Quaterniond q_i_c(t_i_c.unit_quaternion());
  const Eigen::Vector3d p_i_c = t_i_c.translation();
  RCLCPP_INFO(get_logger(),
              "loaded T_i_c: p=[%.6f %.6f %.6f] q_xyzw=[%.6f %.6f %.6f %.6f]",
              p_i_c.x(), p_i_c.y(), p_i_c.z(), q_i_c.x(), q_i_c.y(), q_i_c.z(),
              q_i_c.w());
}

void BasaltNode::buildCalibrationFromParameters() {
  if (image_width_ <= 0 || image_height_ <= 0) {
    throw std::invalid_argument("image_width and image_height must be > 0");
  }

  if (imu_to_cam_translation_.size() != 3) {
    throw std::invalid_argument(
        "imu_to_cam_translation must have exactly 3 elements");
  }

  if (imu_to_cam_rotation_wxyz_.size() != 4) {
    throw std::invalid_argument(
        "imu_to_cam_rotation_wxyz must have exactly 4 elements");
  }

  RCLCPP_INFO(get_logger(),
              "building single-camera %s calibration from ROS parameters",
              camera_model_.c_str());

  calib_ = basalt::Calibration<double>();
  calib_.intrinsics.clear();
  calib_.T_i_c.clear();
  calib_.resolution.clear();
  calib_.vignette.clear();

  calib_.intrinsics.emplace_back(
      basalt::GenericCamera<double>::fromString(camera_model_));
  if (camera_model_ != "pinhole" && camera_model_ != "eucm") {
    throw std::invalid_argument(
        "parameter-built calibration currently supports only pinhole and eucm "
        "camera_model values");
  }
  calib_.intrinsics.back().setFromInit(Eigen::Vector4d(fx_, fy_, cx_, cy_));

  const Eigen::Vector3d translation(imu_to_cam_translation_[0],
                                    imu_to_cam_translation_[1],
                                    imu_to_cam_translation_[2]);
  Eigen::Quaterniond rotation(imu_to_cam_rotation_wxyz_[0],
                              imu_to_cam_rotation_wxyz_[1],
                              imu_to_cam_rotation_wxyz_[2],
                              imu_to_cam_rotation_wxyz_[3]);
  rotation.normalize();

  calib_.T_i_c.emplace_back(rotation, translation);

  Eigen::Vector2i resolution;
  resolution << image_width_, image_height_;
  calib_.resolution.push_back(resolution);
  calib_.vignette.resize(1);
  calib_.imu_update_rate = 250.0;

  RCLCPP_INFO(get_logger(),
              "constructed %s calibration: %dx%d fx=%.6f fy=%.6f cx=%.6f "
              "cy=%.6f",
              camera_model_.c_str(), image_width_, image_height_, fx_, fy_, cx_,
              cy_);
  RCLCPP_INFO(get_logger(),
              "constructed T_i_c: p=[%.6f %.6f %.6f] q_wxyz=[%.6f %.6f %.6f "
              "%.6f]",
              translation.x(), translation.y(), translation.z(), rotation.w(),
              rotation.x(), rotation.y(), rotation.z());
}

void BasaltNode::loadConfig(const std::string &path) {
  requireExistingFile(path, "config_path");
  vio_config_.load(path);
  RCLCPP_INFO(get_logger(), "loaded config from %s", path.c_str());
}
