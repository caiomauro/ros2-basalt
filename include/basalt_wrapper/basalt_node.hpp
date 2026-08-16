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


#pragma once

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <opencv2/core.hpp>

#include <Eigen/Dense>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <basalt/calibration/calibration.hpp>
#include <basalt/camera/generic_camera.hpp>
#include <basalt/image/image.h>
#include <basalt/io/marg_data_io.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/imu_types.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include <tbb/concurrent_queue.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

class BasaltNode final : public rclcpp::Node {
 public:
  using ImageMsg = sensor_msgs::msg::Image;
  using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;

  BasaltNode();
  ~BasaltNode() override;

 private:
  static int64_t rosTimeToNs(const builtin_interfaces::msg::Time &stamp);
  static int64_t steadyNowNs();
  static std::array<double, 36> defaultCovariance(double linear,
                                                  double angular);

  struct PendingImageInput {
    int64_t t_ns{0};
    std::vector<ImageConstPtr> msgs;
    uint64_t seq{0};
    std::string source_topic;
    uint64_t left_hash{0};
    uint64_t right_hash{0};
  };

  struct PendingImuInput {
    int64_t t_ns{0};
    sensor_msgs::msg::Imu::SharedPtr msg;
    uint64_t seq{0};
    std::string source_topic;
  };

  struct PendingStereoComponent {
    ImageConstPtr msg;
    uint64_t seq{0};
    uint64_t hash{0};
    std::string source_topic;
  };

  struct AcceptedBagWrite {
    int64_t t_ns{0};
    uint64_t record_order{0};
    std::string topic;
    ImageConstPtr image;
    sensor_msgs::msg::Imu::SharedPtr imu;
  };

  int64_t imageTimestampNs(const builtin_interfaces::msg::Time &stamp) const;
  bool acceptImageTimestampNs(int64_t candidate_ns, int64_t &accepted_ns);
  bool acceptImuTimestampNs(int64_t candidate_ns, int64_t &accepted_ns);
  bool validateCrossStreamTiming(int64_t image_t_ns, int64_t imu_t_ns);
  void enqueueImageInput(PendingImageInput input);
  void enqueueImuInput(PendingImuInput input);
  void ingestLoop();
  bool dropStaleUnmatchedStereoLocked(int64_t *dropped_t_ns);
  uint64_t hashImageMessage(const ImageConstPtr &msg) const;
  uint64_t hashImuMessage(
      const sensor_msgs::msg::Imu::SharedPtr &msg) const;
  void logIngestPushImu(uint64_t push_idx, const PendingImuInput &imu_input);
  void logIngestPushStereo(uint64_t push_idx, const PendingImageInput &image_input,
                           size_t imu_count_before_image);
  void enqueueAcceptedBagImu(const PendingImuInput &imu_input);
  void enqueueAcceptedBagStereo(const PendingImageInput &image_input);
  void acceptedBagWriterLoop();

  std::string formatVector(const Eigen::Vector3d &v) const;
  std::string formatQuat(const Eigen::Quaterniond &q) const;
  void requireExistingFile(const std::string &path, const char *label) const;

  void loadCalibration(const std::string &path);
  void buildCalibrationFromParameters();
  void loadConfig(const std::string &path);

  basalt::ManagedImage<uint16_t>::Ptr toManagedGray16(const cv::Mat &gray8);
  bool convertToGray(const ImageConstPtr &msg, cv::Mat &gray);
  void publishTrackingImage(const cv::Mat &gray,
                            const std_msgs::msg::Header &header);
  std_msgs::msg::Header headerForTimestamp(int64_t t_ns);
  void rememberImageHeader(int64_t t_ns, const std_msgs::msg::Header &header);
  cv::Mat managedImage16ToMono8(const basalt::ManagedImage<uint16_t> &img);
  cv::Scalar colorForId(size_t id) const;
  void publishOpticalFlowDebug(const basalt::OpticalFlowResult::Ptr &flow_result);
  basalt::OpticalFlowInput::Ptr makeOpticalFlowInput(
      const std::vector<ImageConstPtr> &msgs, int64_t t_ns,
      std_msgs::msg::Header &debug_header);

  void imageCallback(const ImageConstPtr msg);
  void leftImageCallback(const ImageConstPtr msg);
  void rightImageCallback(const ImageConstPtr msg);
  void enqueueStereoComponent(PendingStereoComponent component, bool is_left);
  // Select one real stereo exposure nearest each fixed-rate timestamp slot.
  // Caller must hold ingest_mutex_. Original capture timestamps are retained.
  void enqueueRateSelectedStereoLocked(PendingImageInput input);
  void bagInputLoop();

  void storeLatestImuDebug(const Eigen::Vector3d &raw_gyro,
                           const Eigen::Vector3d &raw_accel,
                           const Eigen::Vector3d &basalt_gyro,
                           const Eigen::Vector3d &basalt_accel,
                           int64_t input_t_ns);
  void applyStartupImuCompensation(Eigen::Vector3d &gyro,
                                   Eigen::Vector3d &accel,
                                   int64_t input_t_ns);
  void maybeAddStartupStationaryAnchor();
  void startupAdjustedMotion(const Eigen::Vector3d &raw_translation,
                             const Eigen::Vector3d &raw_velocity,
                             Eigen::Vector3d &translation,
                             Eigen::Vector3d &velocity);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void globalPositionCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void opticalFlowOutputLoop();
  void estimatorOutputLoop();
  void publishOdometry(const basalt::PoseVelBiasState<double> &state);
  void publishPath(const basalt::PoseVelBiasState<double> &state);
  void recordTrajectory(const basalt::PoseVelBiasState<double> &state);
  void handleDebugSnapshot(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string left_image_topic_;
  std::string right_image_topic_;
  std::string imu_topic_;
  std::string calib_path_;
  std::string config_path_;
  std::string trajectory_output_path_;
  std::string ingest_log_path_;
  std::string diagnostics_log_path_;
  std::string marg_data_output_path_;
  std::string accepted_input_bag_path_;
  int accepted_input_bag_cache_mb_{256};
  std::string global_position_topic_{"/geo/vio_position_measurement"};
  std::string input_mode_{"ros_topics"};
  std::string bag_uri_;
  std::string bag_storage_id_{"sqlite3"};
  int64_t bag_start_time_ns_{0};
  bool bag_preserve_record_order_{false};
  std::string path_frame_id_{"basalt_world"};
  std::string body_frame_id_{"basalt_body"};
  double publish_rate_hz_{100.0};
  int max_path_length_{10000};
  int path_publish_stride_{10};
  bool use_imu_{true};
  bool use_double_{true};
  bool use_camera_info_calibration_{false};
  std::string camera_model_{"pinhole"};
  bool use_header_timestamps_{true};
  bool publish_debug_visuals_{false};
  bool publish_replay_inputs_{false};
  bool use_global_position_factors_{true};
  bool imu_startup_calibration_enabled_{true};
  bool startup_stationary_anchor_enabled_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> input_liveness_faulted_{false};
  int image_width_{1280};
  int image_height_{960};
  int stereo_sync_queue_size_{1000};
  int max_pending_imu_inputs_{2000};
  int max_pending_image_inputs_{250};
  size_t expected_cameras_{1};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  double imu_startup_calibration_sec_{3.0};
  double imu_expected_gravity_mps2_{9.81};
  double imu_startup_max_gyro_std_{0.02};
  double imu_startup_max_accel_std_{0.25};
  double imu_startup_gravity_tolerance_{0.75};
  double startup_stationary_anchor_std_m_{0.03};
  double startup_stationary_anchor_rate_hz_{5.0};
  double startup_motion_gyro_threshold_rad_s_{0.05};
  double startup_motion_accel_delta_threshold_mps2_{0.50};
  double debug_visual_rate_hz_{5.0};
  double max_stereo_input_rate_hz_{0.0};
  double health_max_accel_bias_mps2_{0.75};
  double health_max_gyro_bias_rad_s_{0.03};
  double health_max_speed_mps_{45.0};
  double health_warning_accel_bias_mps2_{0.35};
  double health_warning_accel_bias_slope_mps3_{0.03};
  int health_fault_consecutive_states_{3};
  int health_warning_consecutive_states_{20};
  int health_recovery_consecutive_states_{30};
  int imu_startup_min_samples_{200};
  std::vector<double> imu_to_cam_translation_;
  std::vector<double> imu_to_cam_rotation_wxyz_;

  basalt::Calibration<double> calib_;
  basalt::VioConfig vio_config_;
  basalt::OpticalFlowBase::Ptr opt_flow_;
  basalt::VioEstimatorBase::Ptr vio_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> opt_flow_out_queue_;
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue_;
  basalt::PoseVelBiasState<double>::Ptr latest_state_;
  std::deque<basalt::OpticalFlowResult::Ptr> pending_vision_results_;

  std::mutex latest_imu_mutex_;
  std::mutex latest_state_mutex_;
  std::mutex timestamp_mutex_;
  std::mutex image_headers_mutex_;
  std::mutex ingest_mutex_;
  std::condition_variable latest_imu_cv_;
  std::condition_variable ingest_cv_;
  std::unordered_map<int64_t, std_msgs::msg::Header> image_headers_by_t_ns_;
  std::deque<PendingImageInput> pending_image_inputs_;
  std::deque<PendingImageInput> pending_stereo_rate_candidates_;
  std::deque<PendingImuInput> pending_imu_inputs_;
  std::map<int64_t, PendingStereoComponent> pending_left_images_;
  std::map<int64_t, PendingStereoComponent> pending_right_images_;
  Eigen::Vector3d latest_gyro_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_raw_imu_accel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_raw_imu_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_basalt_accel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_basalt_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d imu_startup_gyro_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d imu_startup_accel_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d imu_startup_gyro_sq_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d imu_startup_accel_sq_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d imu_startup_gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d imu_startup_accel_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startup_stationary_accel_reference_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_raw_pose_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond latest_raw_pose_orientation_{Eigen::Quaterniond::Identity()};
  int64_t latest_imu_t_ns_{0};
  int64_t imu_startup_calibration_start_t_ns_{0};
  int64_t last_image_t_ns_{0};
  int64_t last_imu_input_t_ns_{0};
  bool cross_stream_timing_error_logged_{false};
  bool imu_startup_bias_ready_{false};
  std::atomic<bool> startup_stationary_phase_active_{false};
  std::atomic<bool> estimator_health_faulted_{false};
  int estimator_unhealthy_consecutive_states_{0};
  int estimator_warning_consecutive_states_{0};
  int estimator_healthy_recovery_states_{0};
  int64_t estimator_last_health_t_ns_{0};
  double estimator_last_accel_bias_norm_{0.0};
  double estimator_accel_bias_slope_ewma_mps3_{0.0};
  int startup_motion_consecutive_samples_{0};
  int64_t last_startup_anchor_t_ns_{0};
  size_t startup_anchors_added_{0};
  bool startup_rebase_ready_{false};
  Eigen::Vector3d startup_rebase_translation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d startup_rebase_velocity_{Eigen::Vector3d::Zero()};
  size_t images_received_{0};
  size_t imu_received_{0};
  size_t imu_startup_calibration_samples_{0};
  size_t odom_published_{0};
  size_t path_published_{0};
  std::atomic<int64_t> latest_published_state_t_ns_{0};
  int64_t last_odom_publish_t_ns_{std::numeric_limits<int64_t>::min()};
  size_t last_logged_pending_image_queue_size_{0};
  size_t last_logged_pending_imu_queue_size_{0};
  size_t images_dropped_out_of_order_{0};
  size_t imu_dropped_out_of_order_{0};
  size_t unmatched_stereo_dropped_{0};
  size_t stereo_pairs_rate_limited_{0};
  size_t startup_images_dropped_{0};
  size_t global_positions_received_{0};
  size_t global_positions_rejected_{0};
  size_t stereo_matches_logged_{0};
  int64_t last_stereo_match_delta_ns_{0};
  int64_t next_stereo_input_t_ns_{0};
  int64_t latest_stereo_candidate_t_ns_{0};
  int64_t last_recorded_state_t_ns_{-1};
  int64_t last_debug_visual_t_ns_{std::numeric_limits<int64_t>::min()};
  std::atomic<uint64_t> next_image_source_seq_{0};
  std::atomic<uint64_t> next_imu_source_seq_{0};
  std::atomic<uint64_t> accepted_ingest_events_completed_{0};
  uint64_t next_ingest_push_idx_{0};
  uint64_t imu_pushes_since_last_image_{0};
  std::atomic<bool> accepted_bag_stop_{false};
  std::atomic<bool> accepted_bag_failed_{false};
  uint64_t accepted_bag_images_written_{0};
  uint64_t accepted_bag_imu_written_{0};
  uint64_t accepted_bag_next_record_order_{0};

  std::mutex trajectory_file_mutex_;
  std::ofstream trajectory_file_;
  std::mutex ingest_log_mutex_;
  std::ofstream ingest_log_file_;
  std::mutex diagnostics_log_mutex_;
  std::ofstream diagnostics_log_file_;
  std::mutex accepted_bag_mutex_;
  std::condition_variable accepted_bag_cv_;
  std::deque<AcceptedBagWrite> accepted_bag_queue_;
  std::unique_ptr<rosbag2_cpp::Writer> accepted_bag_writer_;

  basalt::MargDataSaver::Ptr marg_data_saver_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  // Image conversion / stereo bookkeeping can be expensive for full-resolution
  // frames. Keep IMU delivery in its own callback group so camera work cannot
  // starve or decimate the inertial stream.
  rclcpp::CallbackGroup::SharedPtr vision_callback_group_;
  rclcpp::CallbackGroup::SharedPtr right_vision_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      global_position_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pose_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr tracked_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr replay_left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr replay_right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr replay_imu_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr debug_snapshot_srv_;
  nav_msgs::msg::Path path_msg_;
  sensor_msgs::msg::PointCloud pose_cloud_msg_;
  std::thread bag_input_thread_;
  std::thread ingest_thread_;
  std::thread optical_flow_thread_;
  std::thread estimator_thread_;
  std::thread accepted_bag_thread_;
};
