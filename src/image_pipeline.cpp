#include "basalt_wrapper/basalt_node.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>

basalt::ManagedImage<uint16_t>::Ptr BasaltNode::toManagedGray16(
    const cv::Mat &gray8) {
  if (gray8.empty() || gray8.type() != CV_8UC1) {
    throw std::runtime_error("expected non-empty CV_8UC1 grayscale image");
  }

  auto managed =
      std::make_shared<basalt::ManagedImage<uint16_t>>(gray8.cols, gray8.rows);

  for (int row = 0; row < gray8.rows; ++row) {
    const auto *src = gray8.ptr<uint8_t>(row);
    auto *dst = managed->ptr + static_cast<std::ptrdiff_t>(row) * gray8.cols;
    for (int col = 0; col < gray8.cols; ++col) {
      dst[col] = static_cast<uint16_t>(src[col]) << 8;
    }
  }

  return managed;
}

bool BasaltNode::convertToGray(const ImageConstPtr &msg, cv::Mat &gray) {
  try {
    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
      gray = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
      return true;
    }

    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      const auto cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
      return true;
    }

    if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      const auto cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
      return true;
    }
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
    return false;
  }

  RCLCPP_WARN(get_logger(), "unsupported image encoding on image topic: %s",
              msg->encoding.c_str());
  return false;
}

void BasaltNode::publishTrackingImage(const cv::Mat &gray,
                                      const std_msgs::msg::Header &header) {
  if (!tracking_image_pub_) {
    return;
  }
  cv_bridge::CvImage debug_image;
  debug_image.header = header;
  debug_image.encoding = sensor_msgs::image_encodings::MONO8;
  debug_image.image = gray;
  tracking_image_pub_->publish(*debug_image.toImageMsg());
}

std_msgs::msg::Header BasaltNode::headerForTimestamp(int64_t t_ns) {
  std::lock_guard<std::mutex> lock(image_headers_mutex_);
  const auto it = image_headers_by_t_ns_.find(t_ns);
  if (it != image_headers_by_t_ns_.end()) {
    return it->second;
  }
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(t_ns);
  header.frame_id = expected_cameras_ == 2 ? "stereo_camera" : "camera";
  return header;
}

void BasaltNode::rememberImageHeader(int64_t t_ns,
                                     const std_msgs::msg::Header &header) {
  std::lock_guard<std::mutex> lock(image_headers_mutex_);
  image_headers_by_t_ns_[t_ns] = header;
  if (image_headers_by_t_ns_.size() > 300) {
    auto oldest = image_headers_by_t_ns_.begin();
    for (auto it = image_headers_by_t_ns_.begin(); it != image_headers_by_t_ns_.end();
         ++it) {
      if (it->first < oldest->first) {
        oldest = it;
      }
    }
    image_headers_by_t_ns_.erase(oldest);
  }
}

cv::Mat BasaltNode::managedImage16ToMono8(
    const basalt::ManagedImage<uint16_t> &img) {
  cv::Mat out(static_cast<int>(img.h), static_cast<int>(img.w), CV_8UC1);
  for (size_t row = 0; row < img.h; ++row) {
    auto *dst = out.ptr<uint8_t>(static_cast<int>(row));
    const auto *src = img.ptr + static_cast<std::ptrdiff_t>(row) *
                                    static_cast<std::ptrdiff_t>(img.w);
    for (size_t col = 0; col < img.w; ++col) {
      dst[col] = static_cast<uint8_t>(src[col] >> 8);
    }
  }
  return out;
}

cv::Scalar BasaltNode::colorForId(size_t id) const {
  const int r = static_cast<int>((53 * id) % 255);
  const int g = static_cast<int>((97 * id) % 255);
  const int b = static_cast<int>((193 * id) % 255);
  return cv::Scalar(b, g, r);
}

void BasaltNode::publishOpticalFlowDebug(
    const basalt::OpticalFlowResult::Ptr &flow_result) {
  if (!flow_result || flow_result->observations.empty()) {
    return;
  }

  const std_msgs::msg::Header header = headerForTimestamp(flow_result->t_ns);

  if (tracking_overlay_pub_ && flow_result->input_images &&
      !flow_result->input_images->img_data.empty() &&
      flow_result->input_images->img_data[0].img) {
    cv::Mat gray =
        managedImage16ToMono8(*flow_result->input_images->img_data[0].img);
    cv::Mat overlay;
    cv::cvtColor(gray, overlay, cv::COLOR_GRAY2BGR);

    for (size_t cam_idx = 0; cam_idx < flow_result->observations.size(); ++cam_idx) {
      for (const auto &kv : flow_result->observations[cam_idx]) {
        const Eigen::Vector2f px = kv.second.translation();
        cv::circle(overlay,
                   cv::Point(static_cast<int>(std::lround(px.x())),
                             static_cast<int>(std::lround(px.y()))),
                   2, colorForId(kv.first), cv::FILLED, cv::LINE_AA);
      }
    }

    cv_bridge::CvImage overlay_image;
    overlay_image.header = header;
    overlay_image.encoding = sensor_msgs::image_encodings::BGR8;
    overlay_image.image = overlay;
    tracking_overlay_pub_->publish(*overlay_image.toImageMsg());
  }

  if (tracked_points_pub_) {
    sensor_msgs::msg::PointCloud cloud;
    cloud.header = header;

    sensor_msgs::msg::ChannelFloat32 feature_ids;
    feature_ids.name = "feature_id";
    sensor_msgs::msg::ChannelFloat32 camera_ids;
    camera_ids.name = "camera_index";

    for (size_t cam_idx = 0; cam_idx < flow_result->observations.size(); ++cam_idx) {
      for (const auto &kv : flow_result->observations[cam_idx]) {
        const Eigen::Vector2f px = kv.second.translation();
        geometry_msgs::msg::Point32 point;
        point.x = px.x();
        point.y = px.y();
        point.z = 0.0F;
        cloud.points.push_back(point);
        feature_ids.values.push_back(static_cast<float>(kv.first));
        camera_ids.values.push_back(static_cast<float>(cam_idx));
      }
    }

    cloud.channels.push_back(feature_ids);
    cloud.channels.push_back(camera_ids);
    tracked_points_pub_->publish(cloud);
  }
}

basalt::OpticalFlowInput::Ptr BasaltNode::makeOpticalFlowInput(
    const std::vector<ImageConstPtr> &msgs, std_msgs::msg::Header &debug_header) {
  if (msgs.empty()) {
    throw std::invalid_argument("makeOpticalFlowInput received no images");
  }

  auto input = std::make_shared<basalt::OpticalFlowInput>();
  const int64_t candidate_t_ns = imageTimestampNs(msgs.front()->header.stamp);
  if (!acceptImageTimestampNs(candidate_t_ns, input->t_ns)) {
    ++images_dropped_out_of_order_;
    if (images_dropped_out_of_order_ <= 5 ||
        images_dropped_out_of_order_ % 100 == 0) {
      RCLCPP_WARN(
          get_logger(),
          "dropping out-of-order image frame stamp=%lld last_image_t_ns=%lld dropped=%zu",
          static_cast<long long>(candidate_t_ns),
          static_cast<long long>(last_image_t_ns_),
          images_dropped_out_of_order_);
    }
    return nullptr;
  }
  input->img_data.resize(msgs.size());
  debug_header = msgs.front()->header;
  rememberImageHeader(input->t_ns, debug_header);

  for (size_t i = 0; i < msgs.size(); ++i) {
    cv::Mat gray;
    if (!convertToGray(msgs[i], gray)) {
      return nullptr;
    }
    if (i == 0) {
      publishTrackingImage(gray, debug_header);
    }
    input->img_data[i].exposure = 0.0;
    input->img_data[i].img = toManagedGray16(gray);
  }

  return input;
}

void BasaltNode::imageCallback(const ImageConstPtr msg) {
  if (shutting_down_) {
    return;
  }

  try {
    std_msgs::msg::Header debug_header;
    auto input = makeOpticalFlowInput({msg}, debug_header);
    if (!input) {
      return;
    }

    opt_flow_->input_queue.push(input);
    ++images_received_;
    if (images_received_ % 100 == 0) {
      RCLCPP_INFO(
          get_logger(),
          "image frames received=%zu last_image_t_ns=%lld last_imu_t_ns=%lld",
          images_received_, static_cast<long long>(input->t_ns),
          static_cast<long long>(latest_imu_t_ns_));
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "image callback failed: %s", e.what());
  }
}

void BasaltNode::stereoImageCallback(const ImageConstPtr left,
                                     const ImageConstPtr right) {
  if (shutting_down_) {
    return;
  }

  try {
    std_msgs::msg::Header debug_header;
    auto input = makeOpticalFlowInput({left, right}, debug_header);
    if (!input) {
      return;
    }

    opt_flow_->input_queue.push(input);
    ++images_received_;
    if (images_received_ % 100 == 0) {
      RCLCPP_INFO(
          get_logger(),
          "stereo frames received=%zu last_image_t_ns=%lld last_imu_t_ns=%lld",
          images_received_, static_cast<long long>(input->t_ns),
          static_cast<long long>(latest_imu_t_ns_));
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "stereo image callback failed: %s", e.what());
  }
}
