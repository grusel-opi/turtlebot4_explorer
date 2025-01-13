// Copyright (c) 2020 Fetullah Atas
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WAYPOINT_PLUGIN_PHOTO_WAIT_AT_WAYPOINT_HPP_
#define WAYPOINT_PLUGIN_PHOTO_WAIT_AT_WAYPOINT_HPP_

/**
 * While C++17 isn't the project standard. We have to force LLVM/CLang
 * to ignore deprecated declarations
 */
#define _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM

#include <exception>
#include <filesystem>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace waypoint_plugin {

class WaitPhotoAtWaypoint : public nav2_core::WaypointTaskExecutor {
public:
  WaitPhotoAtWaypoint();

  ~WaitPhotoAtWaypoint();

  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                  const std::string &plugin_name);

  bool processAtWaypoint(const geometry_msgs::msg::PoseStamped &curr_pose,
                         const int &curr_waypoint_index);

  void imageCallbackA(const sensor_msgs::msg::Image::SharedPtr msg);
  void imageCallbackB(const sensor_msgs::msg::Image::SharedPtr msg);
  void imageCallbackC(const sensor_msgs::msg::Image::SharedPtr msg);

  static void deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr &msg,
                              cv::Mat &mat);

protected:
  bool is_enabled_;
  bool wait_at_waypoint_;
  int image_amount_;
  int waypoint_pause_duration_;

  rclcpp::Clock::SharedPtr clock_;

  std::string image_format_; // .png ? .jpg ? or some other well known format

  rclcpp::Logger logger_{rclcpp::get_logger("waypoint_plugin")};

  std::mutex global_mutex_A_;
  std::mutex global_mutex_B_;
  std::mutex global_mutex_C_;

  bool is_enabled_A_;
  bool is_enabled_B_;
  bool is_enabled_C_;

  std::filesystem::path save_dir_A_;
  std::filesystem::path save_dir_B_;
  std::filesystem::path save_dir_C_;

  std::string image_topic_A_;
  std::string image_topic_B_;
  std::string image_topic_C_;

  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_A_;
  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_B_;
  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_C_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      camera_image_subscriber_A_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      camera_image_subscriber_B_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      camera_image_subscriber_C_;
};

} // namespace waypoint_plugin

#endif