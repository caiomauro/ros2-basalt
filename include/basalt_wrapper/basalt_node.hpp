#pragma once

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/core.hpp>

#include <Eigen/Dense>

#include <basalt/calibration/calibration.hpp>
#include <basalt/camera/generic_camera.hpp>
#include <basalt/image/image.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/imu_types.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/vio_estimator.h>

#include <tbb/concurrent_queue.h>

#include <array>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
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
  using StereoSync = message_filters::TimeSynchronizer<ImageMsg, ImageMsg>;

  BasaltNode();
  ~BasaltNode() override;

 private:
  static int64_t rosTimeToNs(const builtin_interfaces::msg::Time &stamp);
  static int64_t steadyNowNs();
  static std::array<double, 36> defaultCovariance(double linear,
                                                  double angular);

  int64_t imageTimestampNs(const builtin_interfaces::msg::Time &stamp) const;
  bool acceptImageTimestampNs(int64_t candidate_ns, int64_t &accepted_ns);
  bool acceptImuTimestampNs(int64_t candidate_ns, int64_t &accepted_ns);

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
      const std::vector<ImageConstPtr> &msgs,
      std_msgs::msg::Header &debug_header);

  void imageCallback(const ImageConstPtr msg);
  void stereoImageCallback(const ImageConstPtr left, const ImageConstPtr right);

  void storeLatestImuDebug(const Eigen::Vector3d &raw_gyro,
                           const Eigen::Vector3d &raw_accel,
                           const Eigen::Vector3d &basalt_gyro,
                           const Eigen::Vector3d &basalt_accel,
                           int64_t input_t_ns);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

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
  std::string path_frame_id_{"basalt_world"};
  std::string body_frame_id_{"basalt_body"};
  double publish_rate_hz_{100.0};
  int max_path_length_{10000};
  bool use_imu_{true};
  bool use_double_{false};
  bool use_camera_info_calibration_{false};
  std::string camera_model_{"pinhole"};
  bool use_header_timestamps_{true};
  bool shutting_down_{false};
  int image_width_{1280};
  int image_height_{960};
  size_t expected_cameras_{1};
  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
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
  std::condition_variable latest_imu_cv_;
  std::unordered_map<int64_t, std_msgs::msg::Header> image_headers_by_t_ns_;
  Eigen::Vector3d latest_gyro_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_raw_imu_accel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_raw_imu_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_basalt_accel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_basalt_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_raw_pose_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond latest_raw_pose_orientation_{Eigen::Quaterniond::Identity()};
  int64_t latest_imu_t_ns_{0};
  int64_t last_image_t_ns_{0};
  int64_t last_imu_input_t_ns_{0};
  size_t images_received_{0};
  size_t imu_received_{0};
  size_t odom_published_{0};
  size_t images_dropped_out_of_order_{0};
  size_t imu_dropped_out_of_order_{0};
  int64_t last_recorded_state_t_ns_{-1};

  std::mutex trajectory_file_mutex_;
  std::ofstream trajectory_file_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      left_image_sub_filter_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      right_image_sub_filter_;
  std::shared_ptr<StereoSync> stereo_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pose_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_overlay_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr tracked_points_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr debug_snapshot_srv_;
  nav_msgs::msg::Path path_msg_;
  sensor_msgs::msg::PointCloud pose_cloud_msg_;
  std::thread optical_flow_thread_;
  std::thread estimator_thread_;
};
