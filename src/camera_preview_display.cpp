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


#include "basalt_wrapper/camera_preview_display.hpp"

#include <QGroupBox>
#include <QLabel>
#include <QMetaObject>
#include <QPixmap>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWidget>

#include <functional>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace basalt_wrapper
{

CameraPreviewDisplay::CameraPreviewDisplay() = default;

CameraPreviewDisplay::~CameraPreviewDisplay()
{
  stopSubscriptions();
}

void CameraPreviewDisplay::onInitialize()
{
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  pane_ = new QWidget();
  pane_->setMinimumWidth(390);
  auto * stack = new QVBoxLayout(pane_);
  stack->setContentsMargins(4, 4, 4, 4);
  stack->setSpacing(4);

  const std::array<QString, 3> titles = {
    "Stereo left camera", "Stabilized nadir camera", "Basalt tracking overlay"};
  const std::array<QString, 3> topics = {
    "/cam0/image_raw", "/downward_camera/image_stabilized", "/basalt/tracking_overlay"};
  for (std::size_t i = 0; i < image_labels_.size(); ++i) {
    auto * group = new QGroupBox(titles[i], pane_);
    auto * layout = new QVBoxLayout(group);
    layout->setContentsMargins(2, 2, 2, 2);
    image_labels_[i] = new QLabel("Waiting for " + topics[i] + "...", group);
    image_labels_[i]->setAlignment(Qt::AlignCenter);
    image_labels_[i]->setMinimumHeight(120);
    image_labels_[i]->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    image_labels_[i]->setScaledContents(true);
    image_labels_[i]->setStyleSheet("QLabel { background: #121419; color: #aeb6c2; }");
    layout->addWidget(image_labels_[i]);
    stack->addWidget(group, 1);
  }

  dock_ = context_->getWindowManager()->addPane(
    "Camera previews", pane_, Qt::RightDockWidgetArea, false);
  dock_->setMinimumWidth(410);
  dock_->setFloating(false);
}

void CameraPreviewDisplay::onEnable()
{
  if (dock_) {
    dock_->show();
  }
  startSubscriptions();
}

void CameraPreviewDisplay::onDisable()
{
  stopSubscriptions();
  if (dock_) {
    dock_->hide();
  }
}

void CameraPreviewDisplay::startSubscriptions()
{
  if (!node_ || subscriptions_[0]) {
    return;
  }
  const std::array<std::string, 3> topics = {
    "/cam0/image_raw", "/downward_camera/image_stabilized", "/basalt/tracking_overlay"};
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  for (std::size_t i = 0; i < topics.size(); ++i) {
    subscriptions_[i] = node_->create_subscription<sensor_msgs::msg::Image>(
      topics[i], qos,
      [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {imageCallback(msg, i);});
  }
}

void CameraPreviewDisplay::stopSubscriptions()
{
  for (auto & subscription : subscriptions_) {
    subscription.reset();
  }
}

void CameraPreviewDisplay::imageCallback(
  sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t index)
{
  if (index >= image_labels_.size() || pane_ == nullptr) {
    return;
  }
  try {
    const auto cv_image = cv_bridge::toCvShare(msg);
    cv::Mat rgb;
    if (cv_image->image.channels() == 1) {
      cv::cvtColor(cv_image->image, rgb, cv::COLOR_GRAY2RGB);
    } else if (msg->encoding == sensor_msgs::image_encodings::RGB8 ||
               msg->encoding == sensor_msgs::image_encodings::RGBA8) {
      if (cv_image->image.channels() == 4) {
        cv::cvtColor(cv_image->image, rgb, cv::COLOR_RGBA2RGB);
      } else {
        rgb = cv_image->image;
      }
    } else if (cv_image->image.channels() == 4) {
      cv::cvtColor(cv_image->image, rgb, cv::COLOR_BGRA2RGB);
    } else {
      cv::cvtColor(cv_image->image, rgb, cv::COLOR_BGR2RGB);
    }
    const QImage image(
      rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
    const QImage owned_image = image.copy();
    const auto frame_count = ++frame_counts_[index];
    if (frame_count == 1) {
      RCLCPP_INFO(
        node_->get_logger(), "RViz camera preview received feed %zu: %ux%u encoding=%s",
        index, msg->width, msg->height, msg->encoding.c_str());
    }
    QMetaObject::invokeMethod(
      pane_,
      [this, index, owned_image, frame_count]() {
        if (image_labels_[index]) {
          image_labels_[index]->setPixmap(QPixmap::fromImage(owned_image));
        }
        setStatusStd(
          rviz_common::properties::StatusProperty::Ok,
          "Camera feed " + std::to_string(index + 1),
          "receiving frames: " + std::to_string(frame_count));
      },
      Qt::QueuedConnection);
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_ERROR(node_->get_logger(), "RViz preview conversion failed: %s", error.what());
    setStatusStd(rviz_common::properties::StatusProperty::Error, "Image conversion", error.what());
  }
}

void CameraPreviewDisplay::update(float, float)
{
}

void CameraPreviewDisplay::reset()
{
  rviz_common::Display::reset();
  const std::array<QString, 3> topics = {
    "/cam0/image_raw", "/downward_camera/image_stabilized", "/basalt/tracking_overlay"};
  frame_counts_.fill(0);
  for (std::size_t i = 0; i < image_labels_.size(); ++i) {
    auto * label = image_labels_[i];
    if (label) {
      label->setText("Waiting for " + topics[i] + "...");
    }
  }
}

}  // namespace basalt_wrapper

PLUGINLIB_EXPORT_CLASS(basalt_wrapper::CameraPreviewDisplay, rviz_common::Display)
