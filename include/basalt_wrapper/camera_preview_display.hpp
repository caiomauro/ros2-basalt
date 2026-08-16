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

#include <QImage>

#include <array>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <sensor_msgs/msg/image.hpp>

class QLabel;
class QWidget;

namespace rviz_common
{
class PanelDockWidget;
}

namespace basalt_wrapper
{

class CameraPreviewDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  CameraPreviewDisplay();
  ~CameraPreviewDisplay() override;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onEnable() override;
  void onDisable() override;

private:
  void startSubscriptions();
  void stopSubscriptions();
  void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t index);

  QWidget * pane_{nullptr};
  rviz_common::PanelDockWidget * dock_{nullptr};
  std::array<QLabel *, 3> image_labels_{{nullptr, nullptr, nullptr}};
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 3> subscriptions_;
  std::array<std::size_t, 3> frame_counts_{{0, 0, 0}};
  rclcpp::Node::SharedPtr node_;
};

}  // namespace basalt_wrapper
