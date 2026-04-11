#include "basalt_wrapper/basalt_node.hpp"

#include <cereal/archives/json.hpp>

#include <basalt/serialization/headers_serialization.h>

#include <chrono>
#include <filesystem>
#include <fstream>
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
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
  path_frame_id_ =
      declare_parameter<std::string>("path_frame_id", "basalt_world");
  body_frame_id_ =
      declare_parameter<std::string>("body_frame_id", "basalt_body");
  max_path_length_ = declare_parameter<int>("max_path_length", 10000);
  use_imu_ = declare_parameter<bool>("use_imu", true);
  use_double_ = declare_parameter<bool>("use_double", false);
  use_camera_info_calibration_ =
      declare_parameter<bool>("use_camera_info_calibration", false);
  camera_model_ = declare_parameter<std::string>("camera_model", "pinhole");
  use_header_timestamps_ =
      declare_parameter<bool>("use_header_timestamps", true);
  image_width_ = declare_parameter<int>("image_width", 1280);
  image_height_ = declare_parameter<int>("image_height", 960);
  fx_ = declare_parameter<double>("fx", 539.9363327026367);
  fy_ = declare_parameter<double>("fy", 539.9363708496094);
  cx_ = declare_parameter<double>("cx", 640.0);
  cy_ = declare_parameter<double>("cy", 480.0);
  imu_to_cam_translation_ = declare_parameter<std::vector<double>>(
      "imu_to_cam_translation", {0.12, 0.03, 0.242});
  imu_to_cam_rotation_wxyz_ = declare_parameter<std::vector<double>>(
      "imu_to_cam_rotation_wxyz", {0.5, -0.5, 0.5, -0.5});

  if (publish_rate_hz_ <= 0.0) {
    throw std::invalid_argument("publish_rate_hz must be > 0");
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

  RCLCPP_INFO(get_logger(), "wiring queues");
  opt_flow_->output_queue = &opt_flow_out_queue_;
  vio_->out_state_queue = &out_state_queue_;

  expected_cameras_ = calib_.intrinsics.size();
  if (expected_cameras_ != 1 && expected_cameras_ != 2) {
    throw std::runtime_error(
        "basalt_wrapper currently supports only 1 or 2 cameras in the calibration");
  }

  if (expected_cameras_ == 1) {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        left_image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&BasaltNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "subscribing to mono image topic: %s",
                left_image_topic_.c_str());
  } else {
    if (right_image_topic_.empty()) {
      throw std::invalid_argument(
          "right_image_topic must be set for a 2-camera calibration");
    }
    left_image_sub_filter_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, left_image_topic_);
    right_image_sub_filter_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
            this, right_image_topic_);
    stereo_sync_ = std::make_shared<StereoSync>(
        *left_image_sub_filter_, *right_image_sub_filter_, 100);
    stereo_sync_->registerCallback(std::bind(&BasaltNode::stereoImageCallback,
                                             this, std::placeholders::_1,
                                             std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "subscribing to stereo image topics: %s %s",
                left_image_topic_.c_str(), right_image_topic_.c_str());
  }

  if (use_imu_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&BasaltNode::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "subscribing to IMU topic: %s",
                imu_topic_.c_str());
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/basalt/odometry", rclcpp::SensorDataQoS());
  path_pub_ =
      create_publisher<nav_msgs::msg::Path>("/basalt/path", rclcpp::QoS(10));
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/basalt/pose", rclcpp::QoS(10));
  pose_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud>(
      "/basalt/pose_cloud", rclcpp::QoS(10));
  tracking_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/basalt/tracking_image", rclcpp::QoS(10));
  tracking_overlay_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/basalt/tracking_overlay", rclcpp::QoS(10));
  tracked_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud>(
      "/basalt/tracked_points", rclcpp::QoS(10));

  path_msg_.header.frame_id = path_frame_id_;
  pose_cloud_msg_.header.frame_id = path_frame_id_;

  debug_snapshot_srv_ = create_service<std_srvs::srv::Trigger>(
      "/basalt/get_debug_snapshot",
      std::bind(&BasaltNode::handleDebugSnapshot, this, std::placeholders::_1,
                std::placeholders::_2));

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
  state_timer_ = create_wall_timer(
      period, std::bind(&BasaltNode::processQueuesAndPublish, this));

  RCLCPP_INFO(get_logger(), "subscriptions/publisher ready");
}

BasaltNode::~BasaltNode() {
  shutting_down_ = true;

  try {
    if (opt_flow_) {
      opt_flow_->input_queue.push(nullptr);
    }
    if (vio_) {
      if (use_imu_) {
        vio_->imu_data_queue.push(nullptr);
      }
      vio_->maybe_join();
      vio_->drain_input_queues();
    }
    basalt::PoseVelBiasState<double>::Ptr state;
    while (out_state_queue_.try_pop(state)) {
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

int64_t BasaltNode::nextMonotonicImageTimeNs(int64_t candidate_ns) {
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  if (candidate_ns <= last_image_t_ns_) {
    candidate_ns = last_image_t_ns_ + 1;
  }
  last_image_t_ns_ = candidate_ns;
  return candidate_ns;
}

int64_t BasaltNode::nextMonotonicImuTimeNs(int64_t candidate_ns) {
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  if (candidate_ns <= last_imu_input_t_ns_) {
    candidate_ns = last_imu_input_t_ns_ + 1;
  }
  last_imu_input_t_ns_ = candidate_ns;
  return candidate_ns;
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
