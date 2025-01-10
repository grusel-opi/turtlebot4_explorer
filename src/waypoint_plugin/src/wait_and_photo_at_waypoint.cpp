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

#include "waypoint_plugin/wait_and_photo_at_waypoint.hpp"

#include <string>
#include <memory>
#include <chrono>
#include <ctime>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"

namespace waypoint_plugin
{
WaitPhotoAtWaypoint::WaitPhotoAtWaypoint()
{
}

WaitPhotoAtWaypoint::~WaitPhotoAtWaypoint()
{
}

void WaitPhotoAtWaypoint::initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                     const std::string& plugin_name)
{
  auto node = parent.lock();

  curr_frame_msg_A_ = std::make_shared<sensor_msgs::msg::Image>();
  curr_frame_msg_B_ = std::make_shared<sensor_msgs::msg::Image>();
  curr_frame_msg_C_ = std::make_shared<sensor_msgs::msg::Image>();

  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".waypoint_pause_duration",
                                               rclcpp::ParameterValue(0));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".enabled", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".wait_at_waypoint", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".image_amount", rclcpp::ParameterValue(3));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".enabledA", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".enabledB", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".enabledC", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".image_topic_A",
                                               rclcpp::ParameterValue("/camera/color/image_raw"));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".image_topic_B",
                                               rclcpp::ParameterValue("/camera/color/image_raw"));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".image_topic_C",
                                               rclcpp::ParameterValue("/camera/color/image_raw"));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".save_dir",
                                               rclcpp::ParameterValue("/tmp/waypoint_images"));

  nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".image_format", rclcpp::ParameterValue("png"));

  std::string save_dir_as_string;

  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".enabledA", is_enabled_A_);
  node->get_parameter(plugin_name + ".enabledB", is_enabled_B_);
  node->get_parameter(plugin_name + ".enabledC", is_enabled_C_);
  node->get_parameter(plugin_name + ".wait_at_waypoint", wait_at_waypoint_);
  node->get_parameter(plugin_name + ".image_amount", image_amount_);
  node->get_parameter(plugin_name + ".image_topic_A", image_topic_A_);
  node->get_parameter(plugin_name + ".image_topic_B", image_topic_B_);
  node->get_parameter(plugin_name + ".image_topic_C", image_topic_C_);
  node->get_parameter(plugin_name + ".save_dir", save_dir_as_string);
  node->get_parameter(plugin_name + ".image_format", image_format_);
  node->get_parameter(plugin_name + ".waypoint_pause_duration", waypoint_pause_duration_);

  // get inputted save directory and make sure it exists, if not log and create  it
  save_dir_A_ = save_dir_as_string + "cam_A";
  save_dir_B_ = save_dir_as_string + "cam_B";
  save_dir_C_ = save_dir_as_string + "cam_C";

  RCLCPP_INFO(logger_, "Params:");

  RCLCPP_INFO(logger_, "enabled: %d", is_enabled_);

  RCLCPP_INFO(logger_, "enabledA: %d", is_enabled_A_);
  RCLCPP_INFO(logger_, "enabledB: %d", is_enabled_B_);
  RCLCPP_INFO(logger_, "enabledC: %d", is_enabled_C_);

  RCLCPP_INFO(logger_, "wait_at_waypoint: %d", wait_at_waypoint_);
  RCLCPP_INFO(logger_, "image_amount: %d", image_amount_);

  RCLCPP_INFO(logger_, "image_topic_A: %s", image_topic_A_.c_str());
  RCLCPP_INFO(logger_, "image_topic_B: %s", image_topic_B_.c_str());
  RCLCPP_INFO(logger_, "image_topic_C: %s", image_topic_C_.c_str());

  RCLCPP_INFO(logger_, "save_dir: %s", save_dir_as_string.c_str());

  RCLCPP_INFO(logger_, "image_format: %s", image_format_.c_str());
  RCLCPP_INFO(logger_, "waypoint_pause_duration: %d", waypoint_pause_duration_);

  try
  {
    if (!std::filesystem::exists(save_dir_as_string))
    {
      RCLCPP_WARN(logger_,
                  "Provided save parent directory for photos at waypoint plugin does not exist,"
                  "provided directory is: %s, the directory will be created automatically.",
                  save_dir_as_string.c_str());
      if (!std::filesystem::create_directory(save_dir_as_string))
      {
        RCLCPP_ERROR(logger_,
                     "Failed to create directory!: %s required by photo at waypoint plugin, "
                     "exiting the plugin with failure!",
                     save_dir_as_string.c_str());
        is_enabled_ = false;
      }
    }

    if (!std::filesystem::exists(save_dir_A_))
    {
      RCLCPP_WARN(logger_,
                  "Provided save directory for cam A at waypoint plugin does not exist,"
                  "provided directory is: %s, the directory will be created automatically.",
                  save_dir_A_.c_str());
      if (!std::filesystem::create_directory(save_dir_A_))
      {
        RCLCPP_ERROR(logger_,
                     "Failed to create directory!: %s required by photo at waypoint plugin, "
                     "exiting the plugin with failure!",
                     save_dir_A_.c_str());
        is_enabled_ = false;
      }
    }

    if (!std::filesystem::exists(save_dir_B_))
    {
      RCLCPP_WARN(logger_,
                  "Provided save directory for cam B at waypoint plugin does not exist,"
                  "provided directory is: %s, the directory will be created automatically.",
                  save_dir_B_.c_str());
      if (!std::filesystem::create_directory(save_dir_B_))
      {
        RCLCPP_ERROR(logger_,
                     "Failed to create directory!: %s required by photo at waypoint plugin, "
                     "exiting the plugin with failure!",
                     save_dir_B_.c_str());
        is_enabled_ = false;
      }
    }

    if (!std::filesystem::exists(save_dir_C_))
    {
      RCLCPP_WARN(logger_,
                  "Provided save directory for cam C at waypoint plugin does not exist,"
                  "provided directory is: %s, the directory will be created automatically.",
                  save_dir_C_.c_str());
      if (!std::filesystem::create_directory(save_dir_C_))
      {
        RCLCPP_ERROR(logger_,
                     "Failed to create directory!: %s required by photo at waypoint plugin, "
                     "exiting the plugin with failure!",
                     save_dir_C_.c_str());
        is_enabled_ = false;
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger_,
                 "Exception (%s) thrown while attempting to create image capture directory."
                 " This task executor is being disabled as it cannot save images.",
                 e.what());
    is_enabled_ = false;
  }

  if (!is_enabled_)
  {
    RCLCPP_INFO(logger_, "Photo at waypoint plugin is disabled.");
  }
  else
  {
    if (is_enabled_A_)
    {
      RCLCPP_INFO(logger_, "Initializing photo at waypoint plugin, subscribing to camera topic %s",
                  image_topic_A_.c_str());

      camera_image_subscriber_A_ = node->create_subscription<sensor_msgs::msg::Image>(
          image_topic_A_, rclcpp::SystemDefaultsQoS(),
          std::bind(&WaitPhotoAtWaypoint::imageCallbackA, this, std::placeholders::_1));
    }

    if (is_enabled_B_)
    {
      RCLCPP_INFO(logger_, "Initializing photo at waypoint plugin, subscribing to camera topic %s",
                  image_topic_B_.c_str());

      camera_image_subscriber_B_ = node->create_subscription<sensor_msgs::msg::Image>(
          image_topic_B_, rclcpp::SystemDefaultsQoS(),
          std::bind(&WaitPhotoAtWaypoint::imageCallbackB, this, std::placeholders::_1));
    }

    if (is_enabled_C_)
    {
      RCLCPP_INFO(logger_, "Initializing photo at waypoint plugin, subscribing to camera topic %s",
                  image_topic_C_.c_str());

      camera_image_subscriber_C_ = node->create_subscription<sensor_msgs::msg::Image>(
          image_topic_C_, rclcpp::SystemDefaultsQoS(),
          std::bind(&WaitPhotoAtWaypoint::imageCallbackC, this, std::placeholders::_1));
    }
  }
}

bool WaitPhotoAtWaypoint::processAtWaypoint(const geometry_msgs::msg::PoseStamped& curr_pose,
                                            const int& curr_waypoint_index)
{
  if (!is_enabled_)
  {
    return true;
  }

  auto img_A_stamp = curr_frame_msg_A_->header.stamp;
  auto img_B_stamp = curr_frame_msg_B_->header.stamp;
  auto img_C_stamp = curr_frame_msg_C_->header.stamp;

  if (wait_at_waypoint_)
  {
    RCLCPP_INFO(logger_, "Arrived at %i'th waypoint, waiting for %i images before saving picture..",
                curr_waypoint_index, waypoint_pause_duration_);

    clock_->sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
  }

  auto count = 0;
  auto total_amount = image_amount_ * (is_enabled_A_ + is_enabled_B_ + is_enabled_C_);

  // wait for new images..
  do
  {  // TODO: but maybe not like this..

    if (is_enabled_A_)
    {
      if (curr_frame_msg_A_->header.stamp != img_A_stamp)
      {
        count++;
        img_A_stamp = curr_frame_msg_A_->header.stamp;
        RCLCPP_INFO(logger_, "Got %i of %i image(s)..", count, total_amount);
      }
    }

    if (is_enabled_B_)
    {
      if (curr_frame_msg_B_->header.stamp != img_B_stamp)
      {
        count++;
        img_B_stamp = curr_frame_msg_B_->header.stamp;
        RCLCPP_INFO(logger_, "Got %i of %i image(s)..", count, total_amount);
      }
    }

    if (is_enabled_C_)
    {
      if (curr_frame_msg_C_->header.stamp != img_C_stamp)
      {
        count++;
        img_C_stamp = curr_frame_msg_C_->header.stamp;
        RCLCPP_INFO(logger_, "Got %i of %i image(s)..", count, total_amount);
      }
    }

    clock_->sleep_for(std::chrono::milliseconds(100));

  } while (count < total_amount);

  if (is_enabled_A_)
  {
    try
    {
      std::lock_guard<std::mutex> guard(global_mutex_A_);

      std::filesystem::path file_name_A = std::to_string(curr_waypoint_index) + "_" +
                                          std::to_string(curr_frame_msg_A_->header.stamp.sec) + "." + image_format_;
      std::filesystem::path full_path_image_path_A = save_dir_A_ / file_name_A;

      cv::Mat curr_frame_mat_A;
      deepCopyMsg2Mat(curr_frame_msg_A_, curr_frame_mat_A);
      cv::imwrite(full_path_image_path_A.c_str(), curr_frame_mat_A);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_,
                   "Couldn't take photo at waypoint %i! Caught exception: %s \n"
                   "Make sure that the image topic named: %s is valid and active!",
                   curr_waypoint_index, e.what(), image_topic_A_.c_str());
    }
  }

  if (is_enabled_B_)
  {
    try
    {
      std::filesystem::path file_name_B = std::to_string(curr_waypoint_index) + "_" +
                                          std::to_string(curr_frame_msg_B_->header.stamp.sec) + "." + image_format_;
      std::filesystem::path full_path_image_path_B = save_dir_B_ / file_name_B;

      cv::Mat curr_frame_mat_B;
      deepCopyMsg2Mat(curr_frame_msg_B_, curr_frame_mat_B);
      cv::imwrite(full_path_image_path_B.c_str(), curr_frame_mat_B);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_,
                   "Couldn't take photo at waypoint %i! Caught exception: %s \n"
                   "Make sure that the image topic named: %s is valid and active!",
                   curr_waypoint_index, e.what(), image_topic_B_.c_str());
      return false;
    }
  }

  if (is_enabled_C_)
  {
    try
    {
      std::filesystem::path file_name_C = std::to_string(curr_waypoint_index) + "_" +
                                          std::to_string(curr_frame_msg_C_->header.stamp.sec) + "." + image_format_;
      std::filesystem::path full_path_image_path_C = save_dir_C_ / file_name_C;

      cv::Mat curr_frame_mat_C;
      deepCopyMsg2Mat(curr_frame_msg_C_, curr_frame_mat_C);
      cv::imwrite(full_path_image_path_C.c_str(), curr_frame_mat_C);

      RCLCPP_INFO(logger_, "Photos have been taken sucessfully at waypoint %i", curr_waypoint_index);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_,
                   "Couldn't take photo at waypoint %i! Caught exception: %s \n"
                   "Make sure that the image topic named: %s is valid and active!",
                   curr_waypoint_index, e.what(), image_topic_C_.c_str());
      return false;
    }
  }

  return true;
}

void WaitPhotoAtWaypoint::imageCallbackA(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // RCLCPP_INFO(logger_, "Got a new image with sec stamp %i!", msg->header.stamp.sec);
  std::lock_guard<std::mutex> guard(global_mutex_A_);
  curr_frame_msg_A_ = msg;
}

void WaitPhotoAtWaypoint::imageCallbackB(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // RCLCPP_INFO(logger_, "Got a new image with sec stamp %i!", msg->header.stamp.sec);
  std::lock_guard<std::mutex> guard(global_mutex_B_);
  curr_frame_msg_B_ = msg;
}

void WaitPhotoAtWaypoint::imageCallbackC(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // RCLCPP_INFO(logger_, "Got a new image with sec stamp %i!", msg->header.stamp.sec);
  std::lock_guard<std::mutex> guard(global_mutex_C_);
  curr_frame_msg_C_ = msg;
}

void WaitPhotoAtWaypoint::deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr& msg, cv::Mat& mat)
{
  cv_bridge::CvImageConstPtr cv_bridge_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  cv::Mat frame = cv_bridge_ptr->image;
  if (msg->encoding == "rgb8")
  {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }
  frame.copyTo(mat);
}

}  // namespace waypoint_plugin
PLUGINLIB_EXPORT_CLASS(waypoint_plugin::WaitPhotoAtWaypoint, nav2_core::WaypointTaskExecutor)