#include "basalt_wrapper/basalt_node.hpp"

#include <sstream>

void BasaltNode::handleDebugSnapshot(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  Eigen::Vector3d raw_imu_accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d raw_imu_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d basalt_accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d basalt_gyro = Eigen::Vector3d::Zero();
  int64_t latest_imu_t_ns = 0;
  {
    std::lock_guard<std::mutex> lock(latest_imu_mutex_);
    raw_imu_accel = latest_raw_imu_accel_;
    raw_imu_gyro = latest_raw_imu_gyro_;
    basalt_accel = latest_basalt_accel_;
    basalt_gyro = latest_basalt_gyro_;
    latest_imu_t_ns = latest_imu_t_ns_;
  }

  Eigen::Vector3d raw_pose_translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond raw_pose_orientation = Eigen::Quaterniond::Identity();
  {
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    raw_pose_translation = latest_raw_pose_translation_;
    raw_pose_orientation = latest_raw_pose_orientation_;
  }

  const auto &t_i_c = calib_.T_i_c.front();
  const Eigen::Quaterniond q_i_c(t_i_c.unit_quaternion());
  const Eigen::Vector3d p_i_c = t_i_c.translation();

  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(6);
  oss << "topics: left_image_topic=" << left_image_topic_
      << " right_image_topic=" << right_image_topic_
      << " imu_topic=" << imu_topic_ << "\n";
  oss << "frames: path_frame_id=" << path_frame_id_
      << " body_frame_id=" << body_frame_id_ << "\n";
  oss << "T_i_c.translation=" << formatVector(p_i_c) << "\n";
  oss << "T_i_c.quaternion=" << formatQuat(q_i_c) << "\n";
  oss << "raw_imu_accel=" << formatVector(raw_imu_accel) << "\n";
  oss << "raw_imu_gyro=" << formatVector(raw_imu_gyro) << "\n";
  oss << "basalt_accel=" << formatVector(basalt_accel) << "\n";
  oss << "basalt_gyro=" << formatVector(basalt_gyro) << "\n";
  oss << "latest_imu_t_ns=" << latest_imu_t_ns << "\n";
  oss << "raw_pose.translation=" << formatVector(raw_pose_translation) << "\n";
  oss << "raw_pose.orientation=" << formatQuat(raw_pose_orientation) << "\n";

  response->success = true;
  response->message = oss.str();
}
