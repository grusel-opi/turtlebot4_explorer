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

void WaitPhotoAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();

  curr_frame_msg_ = std::make_shared<sensor_msgs::msg::Image>();

  // logger_ = node->get_logger();

  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".waypoint_pause_duration",
    rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".wait_for_images",
    rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".image_amount",
    rclcpp::ParameterValue(30));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".image_topic",
    rclcpp::ParameterValue("/camera/color/image_raw"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".save_dir",
    rclcpp::ParameterValue("/tmp/waypoint_images"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".image_format",
    rclcpp::ParameterValue("png"));

  std::string save_dir_as_string;
  node->get_parameter(plugin_name + ".enabled", is_enabled_);
  node->get_parameter(plugin_name + ".wait_for_images", wait_for_images_);
  node->get_parameter(plugin_name + ".image_amount", image_amount_);
  node->get_parameter(plugin_name + ".image_topic", image_topic_);
  node->get_parameter(plugin_name + ".save_dir", save_dir_as_string);
  node->get_parameter(plugin_name + ".image_format", image_format_);
  node->get_parameter(plugin_name + ".waypoint_pause_duration", waypoint_pause_duration_);

  // get inputted save directory and make sure it exists, if not log and create  it
  save_dir_ = save_dir_as_string;
  try {
    if (!std::filesystem::exists(save_dir_)) {
      RCLCPP_WARN(
        logger_,
        "Provided save directory for photo at waypoint plugin does not exist,"
        "provided directory is: %s, the directory will be created automatically.",
        save_dir_.c_str()
      );
      if (!std::filesystem::create_directory(save_dir_)) {
        RCLCPP_ERROR(
          logger_,
          "Failed to create directory!: %s required by photo at waypoint plugin, "
          "exiting the plugin with failure!",
          save_dir_.c_str()
        );
        is_enabled_ = false;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_, "Exception (%s) thrown while attempting to create image capture directory."
      " This task executor is being disabled as it cannot save images.", e.what());
    is_enabled_ = false;
  }

  if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Photo at waypoint plugin is disabled.");
  } else {
    RCLCPP_INFO(
      logger_, "Initializing photo at waypoint plugin, subscribing to camera topic named; %s",
      image_topic_.c_str());
    camera_image_subscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&WaitPhotoAtWaypoint::imageCallback, this, std::placeholders::_1));
  }
}

bool WaitPhotoAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }

  auto img_nsec = curr_frame_msg_->header.stamp.nanosec;

  if (wait_for_images_) {

    auto count = 0;

    RCLCPP_INFO(
        logger_, "Arrived at %i'th waypoint, waiting for %i images before saving picture..",
        curr_waypoint_index,
        image_amount_);

    // wait for new images..
    do { // TODO: but maybe not like this..
      if (curr_frame_msg_->header.stamp.nanosec != img_nsec) {
        count++;
        img_nsec = curr_frame_msg_->header.stamp.nanosec;
        RCLCPP_INFO(logger_, "Got %i image(s)..", count);
      }
    } while (count < image_amount_);
    
  } else {

    RCLCPP_INFO(
        logger_, "Arrived at %i'th waypoint, sleeping for %i images before saving picture..",
        curr_waypoint_index,
        waypoint_pause_duration_);

    clock_->sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
    while (curr_frame_msg_->header.stamp.nanosec == img_nsec) {
      clock_->sleep_for(std::chrono::milliseconds(200));
    };
  }

  try {
    std::lock_guard<std::mutex> guard(global_mutex_);

    std::filesystem::path file_name = std::to_string(
      curr_waypoint_index) + "_" +
      std::to_string(curr_frame_msg_->header.stamp.sec) + "." + image_format_;
    std::filesystem::path full_path_image_path = save_dir_ / file_name;

    cv::Mat curr_frame_mat;
    deepCopyMsg2Mat(curr_frame_msg_, curr_frame_mat);
    cv::imwrite(full_path_image_path.c_str(), curr_frame_mat);
    RCLCPP_INFO(
      logger_,
      "Photo has been taken sucessfully at waypoint %i", curr_waypoint_index);
      
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger_,
      "Couldn't take photo at waypoint %i! Caught exception: %s \n"
      "Make sure that the image topic named: %s is valid and active!",
      curr_waypoint_index,
      e.what(), image_topic_.c_str());
    return false;
  }
  return true;
}

void WaitPhotoAtWaypoint::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  //RCLCPP_INFO(logger_, "Got a new image with sec stamp %i!", msg->header.stamp.sec);
  std::lock_guard<std::mutex> guard(global_mutex_);
  curr_frame_msg_ = msg;
}

void WaitPhotoAtWaypoint::deepCopyMsg2Mat(
  const sensor_msgs::msg::Image::SharedPtr & msg,
  cv::Mat & mat)
{
  cv_bridge::CvImageConstPtr cv_bridge_ptr = cv_bridge::toCvShare(msg, msg->encoding);
  cv::Mat frame = cv_bridge_ptr->image;
  if (msg->encoding == "rgb8") {
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
  }
  frame.copyTo(mat);
}

}      // namespace waypoint_plugin
PLUGINLIB_EXPORT_CLASS(
  waypoint_plugin::WaitPhotoAtWaypoint,
  nav2_core::WaypointTaskExecutor)