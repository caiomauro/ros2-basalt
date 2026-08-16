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
      gray = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
      return true;
    }

    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      const auto cv_ptr =
          cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_RGB2GRAY);
      return true;
    }

    if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      const auto cv_ptr =
          cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
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
  if (!flow_result) {
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

    // The overlay background is camera 0. Drawing camera 1 coordinates on the
    // left image makes valid right-camera tracks look misplaced and prevents
    // reliable visual inspection of vehicle-body contamination.
    if (!flow_result->observations.empty()) {
      for (const auto &kv : flow_result->observations[0]) {
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
    const std::vector<ImageConstPtr> &msgs, int64_t t_ns,
    std_msgs::msg::Header &debug_header) {
  if (msgs.empty()) {
    throw std::invalid_argument("makeOpticalFlowInput received no images");
  }

  auto input = std::make_shared<basalt::OpticalFlowInput>();
  if (!acceptImageTimestampNs(t_ns, input->t_ns)) {
    ++images_dropped_out_of_order_;
    if (images_dropped_out_of_order_ <= 5 ||
        images_dropped_out_of_order_ % 100 == 0) {
      RCLCPP_WARN(
          get_logger(),
          "dropping out-of-order image frame stamp=%lld last_image_t_ns=%lld dropped=%zu",
          static_cast<long long>(t_ns),
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
    enqueueImageInput(
        PendingImageInput{imageTimestampNs(msg->header.stamp),
                          {msg},
                          ++next_image_source_seq_,
                          left_image_topic_,
                          hashImageMessage(msg),
                          0});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "image callback failed: %s", e.what());
  }
}

void BasaltNode::enqueueRateSelectedStereoLocked(PendingImageInput input) {
  auto enqueue_ready = [&](PendingImageInput ready) {
    const auto insert_it = std::upper_bound(
        pending_image_inputs_.begin(), pending_image_inputs_.end(), ready.t_ns,
        [](int64_t stamp, const PendingImageInput &candidate) {
          return stamp < candidate.t_ns;
        });
    pending_image_inputs_.insert(insert_it, std::move(ready));
  };

  if (max_stereo_input_rate_hz_ <= 0.0) {
    enqueue_ready(std::move(input));
    return;
  }

  const int64_t period_ns = std::max<int64_t>(
      1, static_cast<int64_t>(
             std::llround(1.0e9 / max_stereo_input_rate_hz_)));
  const int64_t half_period_ns = period_ns / 2;
  const int64_t reorder_horizon_ns = period_ns;

  if (next_stereo_input_t_ns_ == 0) {
    // Anchor the output lattice to a real exposure. Subsequent output slots
    // remain exactly period_ns apart; selected images keep their sensor stamp.
    next_stereo_input_t_ns_ = input.t_ns;
  }

  // A late callback for a slot that was already finalized cannot be used
  // without sending Basalt an out-of-order image.
  if (input.t_ns < next_stereo_input_t_ns_ - half_period_ns) {
    ++stereo_pairs_rate_limited_;
    return;
  }

  latest_stereo_candidate_t_ns_ =
      std::max(latest_stereo_candidate_t_ns_, input.t_ns);
  const auto candidate_it = std::upper_bound(
      pending_stereo_rate_candidates_.begin(),
      pending_stereo_rate_candidates_.end(), input.t_ns,
      [](int64_t stamp, const PendingImageInput &candidate) {
        return stamp < candidate.t_ns;
      });
  pending_stereo_rate_candidates_.insert(candidate_it, std::move(input));

  // One period of reorder look-ahead allows the matching exposure from the
  // other DDS image stream to arrive before a slot is finalized. The selected
  // image itself remains constrained to +/- half a period around the slot and
  // keeps its original sensor timestamp. With redundant simulator exposures,
  // selected gaps stay close to the 50 ms estimator cadence.
  while (latest_stereo_candidate_t_ns_ >=
         next_stereo_input_t_ns_ + reorder_horizon_ns) {
    const int64_t window_start =
        next_stereo_input_t_ns_ - half_period_ns;
    const int64_t window_end = next_stereo_input_t_ns_ + half_period_ns;

    while (!pending_stereo_rate_candidates_.empty() &&
           pending_stereo_rate_candidates_.front().t_ns < window_start) {
      pending_stereo_rate_candidates_.pop_front();
      ++stereo_pairs_rate_limited_;
    }

    auto best_it = pending_stereo_rate_candidates_.end();
    int64_t best_distance_ns = std::numeric_limits<int64_t>::max();
    for (auto it = pending_stereo_rate_candidates_.begin();
         it != pending_stereo_rate_candidates_.end() && it->t_ns <= window_end;
         ++it) {
      const int64_t distance_ns =
          std::llabs(it->t_ns - next_stereo_input_t_ns_);
      if (distance_ns < best_distance_ns) {
        best_distance_ns = distance_ns;
        best_it = it;
      }
    }

    // Rendering / DDS can occasionally omit every ideal-window exposure even
    // though the redundant source has already supplied a later synchronized
    // pair. Use the closest unused real pair rather than skipping an estimator
    // slot. Its original timestamp remains intact; no frame is duplicated or
    // retimestamped.
    if (best_it == pending_stereo_rate_candidates_.end() &&
        !pending_stereo_rate_candidates_.empty()) {
      best_it = pending_stereo_rate_candidates_.begin();
    }

    if (best_it != pending_stereo_rate_candidates_.end()) {
      PendingImageInput selected = std::move(*best_it);
      pending_stereo_rate_candidates_.erase(best_it);
      enqueue_ready(std::move(selected));
    } else {
      // No unused source exposure exists yet. Keep this slot pending until a
      // later callback provides one instead of permanently reducing the rate.
      break;
    }

    next_stereo_input_t_ns_ += period_ns;

    // Retain a boundary candidate for the following slot; discard all other
    // unselected exposures whose nearest output slot is now in the past.
    const int64_t next_window_start =
        next_stereo_input_t_ns_ - half_period_ns;
    while (!pending_stereo_rate_candidates_.empty() &&
           pending_stereo_rate_candidates_.front().t_ns < next_window_start) {
      pending_stereo_rate_candidates_.pop_front();
      ++stereo_pairs_rate_limited_;
    }
  }
}

void BasaltNode::enqueueStereoComponent(PendingStereoComponent component,
                                        bool is_left) {
  // Stereo observations must represent the same exposure.  A 30 ms window
  // can pair adjacent frames at 60 Hz after one bridge message is dropped,
  // turning vehicle motion into false disparity.  Gazebo's two cameras share
  // the simulation clock and normally have identical stamps; retain 1 ms only
  // for transport / timestamp quantization tolerance.
  constexpr int64_t kStereoMatchToleranceNs = 1 * 1000 * 1000;  // 1 ms

  const int64_t t_ns = imageTimestampNs(component.msg->header.stamp);
  std::lock_guard<std::mutex> lock(ingest_mutex_);

  auto &own_map = is_left ? pending_left_images_ : pending_right_images_;
  auto &other_map = is_left ? pending_right_images_ : pending_left_images_;

  if (component.seq <= 2) {
    RCLCPP_INFO(
        get_logger(),
        "%s image arrival seq=%llu t_ns=%lld own_pending=%zu other_pending=%zu encoding=%s",
        is_left ? "left" : "right",
        static_cast<unsigned long long>(component.seq),
        static_cast<long long>(t_ns), own_map.size(), other_map.size(),
        component.msg->encoding.c_str());
  }

  own_map[t_ns] = std::move(component);

  // A dropped or duplicated camera component must not remain forever. Keep a
  // short timestamp window and a hard size bound so long simulations cannot
  // accumulate orphaned full-resolution images or slow stereo lookup.
  constexpr int64_t kStereoRetentionNs = 200 * 1000 * 1000;  // 200 ms
  const int64_t oldest_useful_t_ns = t_ns - kStereoRetentionNs;
  auto prune = [&](auto &pending) {
    while (!pending.empty() &&
           (pending.begin()->first < oldest_useful_t_ns ||
            pending.size() > static_cast<size_t>(stereo_sync_queue_size_))) {
      pending.erase(pending.begin());
      ++unmatched_stereo_dropped_;
    }
  };
  prune(own_map);
  prune(other_map);

  auto own_it = own_map.find(t_ns);
  if (own_it == own_map.end()) {
    ingest_cv_.notify_one();
    return;
  }

  auto best_other_it = other_map.end();
  int64_t best_delta_ns = kStereoMatchToleranceNs + 1;

  auto consider_other = [&](decltype(other_map.begin()) candidate_it) {
    if (candidate_it == other_map.end()) {
      return;
    }
    const int64_t candidate_t_ns = candidate_it->first;
    const int64_t delta_ns = std::llabs(candidate_t_ns - t_ns);
    if (delta_ns <= kStereoMatchToleranceNs && delta_ns < best_delta_ns) {
      best_delta_ns = delta_ns;
      best_other_it = candidate_it;
    }
  };

  const auto lower_it = other_map.lower_bound(t_ns);
  consider_other(lower_it);
  if (lower_it != other_map.begin()) {
    consider_other(std::prev(lower_it));
  }

  if (best_other_it != other_map.end()) {
    const int64_t pair_t_ns = std::max(t_ns, best_other_it->first);
    PendingImageInput input;
    input.t_ns = pair_t_ns;
    last_stereo_match_delta_ns_ = std::llabs(best_other_it->first - t_ns);
    if (is_left) {
      input.msgs = {own_it->second.msg, best_other_it->second.msg};
      input.seq = own_it->second.seq;
      input.source_topic =
          own_it->second.source_topic + "+" + best_other_it->second.source_topic;
      input.left_hash = own_it->second.hash;
      input.right_hash = best_other_it->second.hash;
    } else {
      input.msgs = {best_other_it->second.msg, own_it->second.msg};
      input.seq = best_other_it->second.seq;
      input.source_topic =
          best_other_it->second.source_topic + "+" + own_it->second.source_topic;
      input.left_hash = best_other_it->second.hash;
      input.right_hash = own_it->second.hash;
    }

    enqueueRateSelectedStereoLocked(std::move(input));
    if (stereo_matches_logged_ < 5 || stereo_matches_logged_ % 500 == 0) {
      RCLCPP_INFO(
          get_logger(),
          "matched stereo pair #%zu left_t_ns=%lld right_t_ns=%lld delta_ns=%lld ready_queue=%zu left_pending=%zu right_pending=%zu",
          stereo_matches_logged_ + 1,
          static_cast<long long>(is_left ? t_ns : best_other_it->first),
          static_cast<long long>(is_left ? best_other_it->first : t_ns),
          static_cast<long long>(last_stereo_match_delta_ns_),
          pending_image_inputs_.size(), pending_left_images_.size(),
          pending_right_images_.size());
    }
    ++stereo_matches_logged_;
    own_map.erase(own_it);
    other_map.erase(best_other_it);
  }

  ingest_cv_.notify_one();
}

void BasaltNode::leftImageCallback(const ImageConstPtr msg) {
  if (shutting_down_) {
    return;
  }

  try {
    enqueueStereoComponent(
        PendingStereoComponent{msg, ++next_image_source_seq_, 0,
                               left_image_topic_},
        true);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "left image callback failed: %s", e.what());
  }
}

void BasaltNode::rightImageCallback(const ImageConstPtr msg) {
  if (shutting_down_) {
    return;
  }

  try {
    enqueueStereoComponent(
        PendingStereoComponent{msg, ++next_image_source_seq_, 0,
                               right_image_topic_},
        false);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "right image callback failed: %s", e.what());
  }
}
