#include <exception>
#include <filesystem>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>

namespace turtlebot4_photographer {

class Photographer : public rclcpp::Node {
public:
  Photographer(const rclcpp::NodeOptions & options) : Node("turtlebot4_photographer", options) {
    RCLCPP_INFO(get_logger(), "Turtlebot4 photographer startup.");

    curr_frame_msg_A_ = std::make_shared<sensor_msgs::msg::Image>();
    curr_frame_msg_B_ = std::make_shared<sensor_msgs::msg::Image>();
    curr_frame_msg_C_ = std::make_shared<sensor_msgs::msg::Image>();

    clock_ = get_clock();

    declare_parameter("enabled", rclcpp::ParameterValue(true));

    declare_parameter("trigger_topic",
                      rclcpp::ParameterValue("/waypoint_follower/input_output_at_waypoint/output"));

    declare_parameter("waypoint_pause_duration", rclcpp::ParameterValue(0));

    declare_parameter("wait_at_waypoint", rclcpp::ParameterValue(true));

    declare_parameter("enabledA", rclcpp::ParameterValue(true));

    declare_parameter("enabledB", rclcpp::ParameterValue(true));

    declare_parameter("enabledC", rclcpp::ParameterValue(true));

    declare_parameter("image_topic_A",
                      rclcpp::ParameterValue("/camera_A/color/image_raw"));

    declare_parameter("image_topic_B",
                      rclcpp::ParameterValue("/camera_B/color/image_raw"));

    declare_parameter("image_topic_C",
                      rclcpp::ParameterValue("/camera_C/color/image_raw"));

    declare_parameter("save_dir",
                      rclcpp::ParameterValue("/tmp/waypoint_images"));

    declare_parameter("image_format", rclcpp::ParameterValue("png"));

    std::string save_dir_as_string;

    get_parameter("trigger_topic", trigger_topic_);
    get_parameter("enabled", is_enabled_);
    get_parameter("enabledA", is_enabled_A_);
    get_parameter("enabledB", is_enabled_B_);
    get_parameter("enabledC", is_enabled_C_);
    get_parameter("wait_at_waypoint", wait_at_waypoint_);
    get_parameter("image_topic_A", image_topic_A_);
    get_parameter("image_topic_B", image_topic_B_);
    get_parameter("image_topic_C", image_topic_C_);
    get_parameter("save_dir", save_dir_as_string);
    get_parameter("image_format", image_format_);
    get_parameter("waypoint_pause_duration", waypoint_pause_duration_);

    // get inputted save directory and make sure it exists, if not log and
    // create  it
    save_dir_A_ = save_dir_as_string + "cam_A";
    save_dir_B_ = save_dir_as_string + "cam_B";
    save_dir_C_ = save_dir_as_string + "cam_C";

    RCLCPP_INFO(logger_, "Params:");

    RCLCPP_INFO(logger_, "trigger_topic: %s", trigger_topic_.c_str());

    RCLCPP_INFO(logger_, "enabled: %d", is_enabled_);

    RCLCPP_INFO(logger_, "enabledA: %d", is_enabled_A_);
    RCLCPP_INFO(logger_, "enabledB: %d", is_enabled_B_);
    RCLCPP_INFO(logger_, "enabledC: %d", is_enabled_C_);

    RCLCPP_INFO(logger_, "wait_at_waypoint: %d", wait_at_waypoint_);

    RCLCPP_INFO(logger_, "image_topic_A: %s", image_topic_A_.c_str());
    RCLCPP_INFO(logger_, "image_topic_B: %s", image_topic_B_.c_str());
    RCLCPP_INFO(logger_, "image_topic_C: %s", image_topic_C_.c_str());

    RCLCPP_INFO(logger_, "save_dir: %s", save_dir_as_string.c_str());

    RCLCPP_INFO(logger_, "image_format: %s", image_format_.c_str());
    RCLCPP_INFO(logger_, "waypoint_pause_duration: %d",
                waypoint_pause_duration_);

    try {
      if (!std::filesystem::exists(save_dir_as_string)) {
        RCLCPP_WARN(logger_,
                    "Provided save parent directory for images "
                    "does not exist,"
                    "provided directory is: %s, the directory will be created "
                    "automatically.",
                    save_dir_as_string.c_str());
        if (!std::filesystem::create_directory(save_dir_as_string)) {
          RCLCPP_ERROR(logger_,
                       "Failed to create directory!: %s required by "
                       "Turtlebot4_photographer!",
                       save_dir_as_string.c_str());
          is_enabled_ = false;
        }
      }

      if (is_enabled_A_) {
        if (!std::filesystem::exists(save_dir_A_)) {
          RCLCPP_WARN(
              logger_,
              "Provided save parent directory for cam A "
              "does not exist,"
              "provided directory is: %s, the directory will be created "
              "automatically.",
              save_dir_A_.c_str());
          if (!std::filesystem::create_directory(save_dir_A_)) {
            RCLCPP_ERROR(logger_,
                         "Failed to create directory!: %s required by "
                         "Turtlebot4_photographer!",
                         save_dir_A_.c_str());
            is_enabled_ = false;
          }
        }
      }

      if (is_enabled_B_) {
        if (!std::filesystem::exists(save_dir_B_)) {
          RCLCPP_WARN(
              logger_,
              "Provided save parent directory for cam B "
              "does not exist,"
              "provided directory is: %s, the directory will be created "
              "automatically.",
              save_dir_B_.c_str());
          if (!std::filesystem::create_directory(save_dir_B_)) {
            RCLCPP_ERROR(logger_,
                         "Failed to create directory!: %s required by "
                         "Turtlebot4_photographer!",
                         save_dir_B_.c_str());
            is_enabled_ = false;
          }
        }
      }

      if (is_enabled_C_) {
        if (!std::filesystem::exists(save_dir_C_)) {
          RCLCPP_WARN(
              logger_,
              "Provided save parent directory for cam C "
              "does not exist,"
              "provided directory is: %s, the directory will be created "
              "automatically.",
              save_dir_C_.c_str());
          if (!std::filesystem::create_directory(save_dir_C_)) {
            RCLCPP_ERROR(logger_,
                         "Failed to create directory!: %s required by "
                         "Turtlebot4_photographer!",
                         save_dir_C_.c_str());
            is_enabled_ = false;
          }
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          logger_,
          "Exception (%s) thrown while attempting to create image capture "
          "directory."
          " This task executor is being disabled as it cannot save images.",
          e.what());
      is_enabled_ = false;
    }

    trigger_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        trigger_topic_, 1,
        std::bind(&Photographer::triggerCallback, this, std::placeholders::_1));

    done_publisher_ = create_publisher<std_msgs::msg::Empty>(
      "/waypoint_follower/input_output_at_waypoint/input", 10);

    if (!is_enabled_) {
      RCLCPP_INFO(logger_, "Photo at waypoint plugin is disabled.");
    } else {
      if (is_enabled_A_) {
        RCLCPP_INFO(logger_,
                    "Initializing photo at waypoint plugin, subscribing to "
                    "camera topic %s",
                    image_topic_A_.c_str());

        camera_image_subscriber_A_ =
            create_subscription<sensor_msgs::msg::Image>(
                image_topic_A_, rclcpp::SystemDefaultsQoS(),
                std::bind(&Photographer::imageCallbackA, this,
                          std::placeholders::_1));
      }

      if (is_enabled_B_) {
        RCLCPP_INFO(logger_,
                    "Initializing photo at waypoint plugin, subscribing to "
                    "camera topic %s",
                    image_topic_B_.c_str());

        camera_image_subscriber_B_ =
            create_subscription<sensor_msgs::msg::Image>(
                image_topic_B_, rclcpp::SystemDefaultsQoS(),
                std::bind(&Photographer::imageCallbackB, this,
                          std::placeholders::_1));
      }

      if (is_enabled_C_) {
        RCLCPP_INFO(logger_,
                    "Initializing photo at waypoint plugin, subscribing to "
                    "camera topic %s",
                    image_topic_C_.c_str());

        camera_image_subscriber_C_ =
            create_subscription<sensor_msgs::msg::Image>(
                image_topic_C_, rclcpp::SystemDefaultsQoS(),
                std::bind(&Photographer::imageCallbackC, this,
                          std::placeholders::_1));
      }
    }
  }

  void triggerCallback(const geometry_msgs::msg::PoseStamped &curr_pose) {

    if (wait_at_waypoint_) {
      RCLCPP_INFO(logger_,
                  "Arrived at waypoint at pos (%f, %f), sleeping for %i "
                  "ms before saving picture..",
                  curr_pose.pose.position.x,
                  curr_pose.pose.position.y, waypoint_pause_duration_);

      clock_->sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
    }

    if (is_enabled_A_) {
      try {
        std::filesystem::path file_name_A =
            std::to_string(curr_pose.header.stamp.sec) + "." + image_format_;
        std::filesystem::path full_path_image_path_A =
            save_dir_A_ / file_name_A;

        std::lock_guard<std::mutex> guard(global_mutex_A_);
        cv::Mat curr_frame_mat_A;
        deepCopyMsg2Mat(curr_frame_msg_A_, curr_frame_mat_A);
        cv::imwrite(full_path_image_path_A.c_str(), curr_frame_mat_A);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(
            logger_,
            "Couldn't take photo at waypoint! Caught exception: %s \n"
            "Make sure that the image topic named: %s is valid and active!",
            e.what(), image_topic_A_.c_str());
      }
    }

    if (is_enabled_B_) {
      try {
        std::filesystem::path file_name_B =
            std::to_string(curr_pose.header.stamp.sec) + "." + image_format_;
        std::filesystem::path full_path_image_path_B =
            save_dir_B_ / file_name_B;

        std::lock_guard<std::mutex> guard(global_mutex_B_);
        cv::Mat curr_frame_mat_B;
        deepCopyMsg2Mat(curr_frame_msg_B_, curr_frame_mat_B);
        cv::imwrite(full_path_image_path_B.c_str(), curr_frame_mat_B);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(
            logger_,
            "Couldn't take photo at waypoint! Caught exception: %s \n"
            "Make sure that the image topic named: %s is valid and active!",
            e.what(), image_topic_B_.c_str());
      }
    }

    if (is_enabled_C_) {
      try {
        std::filesystem::path file_name_C =
            std::to_string(curr_pose.header.stamp.sec) + "." + image_format_;
        std::filesystem::path full_path_image_path_C =
            save_dir_C_ / file_name_C;

        std::lock_guard<std::mutex> guard(global_mutex_C_);
        cv::Mat curr_frame_mat_C;
        deepCopyMsg2Mat(curr_frame_msg_C_, curr_frame_mat_C);
        cv::imwrite(full_path_image_path_C.c_str(), curr_frame_mat_C);

        RCLCPP_INFO(logger_,
                    "Photos have been taken sucessfully at waypoint");
      } catch (const std::exception &e) {
        RCLCPP_ERROR(
            logger_,
            "Couldn't take photo at waypoint! Caught exception: %s \n"
            "Make sure that the image topic named: %s is valid and active!",
            e.what(), image_topic_C_.c_str());
      }
    }

    std_msgs::msg::Empty done_trigger;

    done_publisher_->publish(done_trigger);

  }

  void imageCallbackA(const sensor_msgs::msg::Image::SharedPtr msg) {
    // RCLCPP_INFO(logger_, "Got a new image with sec stamp %i!",
    // msg->header.stamp.sec);
    std::lock_guard<std::mutex> guard(global_mutex_A_);
    curr_frame_msg_A_ = msg;
  }

  void imageCallbackB(const sensor_msgs::msg::Image::SharedPtr msg) {
    // RCLCPP_INFO(logger_, "Got a new image with sec stamp %i!",
    // msg->header.stamp.sec);
    std::lock_guard<std::mutex> guard(global_mutex_B_);
    curr_frame_msg_B_ = msg;
  }

  void imageCallbackC(const sensor_msgs::msg::Image::SharedPtr msg) {
    // RCLCPP_INFO(logger_, "Got a new image with sec stamp %i!",
    // msg->header.stamp.sec);
    std::lock_guard<std::mutex> guard(global_mutex_C_);
    curr_frame_msg_C_ = msg;
  }

  void deepCopyMsg2Mat(const sensor_msgs::msg::Image::SharedPtr &msg,
                       cv::Mat &mat) {
    cv_bridge::CvImageConstPtr cv_bridge_ptr =
        cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat frame = cv_bridge_ptr->image;
    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }
    frame.copyTo(mat);
  }

private:
  std::string trigger_topic_;

  std::string image_format_; // .png ? .jpg ? or some other well known format

  rclcpp::Logger logger_{rclcpp::get_logger("waypoint_plugin")};

  rclcpp::Clock::SharedPtr clock_;

  bool wait_at_waypoint_;
  int waypoint_pause_duration_;

  bool is_enabled_;

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

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_subscriber_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(turtlebot4_photographer::Photographer)

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Photographer>());
//   rclcpp::shutdown();
//   return 0;
// }