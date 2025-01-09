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


#include <filesystem>
#include <mutex>
#include <string>
#include <exception>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"


namespace waypoint_plugin
{

class WaitPhotoAtWaypoint : public nav2_core::WaypointTaskExecutor
{
public:
  
  WaitPhotoAtWaypoint();

  ~WaitPhotoAtWaypoint();

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name);

  bool processAtWaypoint(const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  static void deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & mat);

protected:

  std::mutex global_mutex_;

  std::filesystem::path save_dir_;
  
  std::string image_format_; // .png ? .jpg ? or some other well known format

  std::string image_topic_;

  bool is_enabled_;

  bool wait_for_images_;

  int image_amount_;

  int waypoint_pause_duration_;

  rclcpp::Clock::SharedPtr clock_;

  sensor_msgs::msg::Image::SharedPtr curr_frame_msg_;

  rclcpp::Logger logger_{rclcpp::get_logger("waypoint_plugin")};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_subscriber_;
};

}

#endif