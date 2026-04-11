#include "basalt_wrapper/basalt_node.hpp"

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

  auto imu = std::make_shared<basalt::ImuData<double>>();
  const int64_t imu_t_ns =
      use_header_timestamps_ ? rosTimeToNs(msg->header.stamp) : steadyNowNs();
  imu->t_ns = nextMonotonicImuTimeNs(imu_t_ns);

  const Eigen::Vector3d raw_gyro(msg->angular_velocity.x,
                                 msg->angular_velocity.y,
                                 msg->angular_velocity.z);
  const Eigen::Vector3d raw_accel(msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);
  imu->gyro = raw_gyro;
  imu->accel = raw_accel;

  storeLatestImuDebug(raw_gyro, raw_accel, imu->gyro, imu->accel, imu->t_ns);

  vio_->imu_data_queue.push(imu);
  ++imu_received_;
}
