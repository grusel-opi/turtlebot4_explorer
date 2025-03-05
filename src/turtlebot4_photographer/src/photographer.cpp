#include <exception>
#include <filesystem>
#include <memory>
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
#include <vector>

#include "turtlebot4_photographer/util.hpp"

namespace turtlebot4_photographer {

class Photographer : public rclcpp::Node {
public:
  Photographer(const rclcpp::NodeOptions &options)
      : Node("turtlebot4_photographer", options) {
    RCLCPP_INFO(get_logger(), "Turtlebot4 photographer startup.");

    clock_ = get_clock();

    declare_parameter("enabled", rclcpp::ParameterValue(true));

    declare_parameter("trigger_topic", rclcpp::ParameterValue(
                                           "/input_output_at_waypoint/output"));

    declare_parameter("waypoint_pause_duration", rclcpp::ParameterValue(0));

    declare_parameter("wait_at_waypoint", rclcpp::ParameterValue(true));

    declare_parameter("save_dir",
                      rclcpp::ParameterValue("/tmp/waypoint_images"));

    declare_parameter("image_format", rclcpp::ParameterValue("png"));

    declare_parameter("topics",
                      rclcpp::ParameterValue(std::vector<std::string>()));

    std::string save_dir_as_string;

    get_parameter("trigger_topic", trigger_topic_);
    get_parameter("enabled", is_enabled_);
    get_parameter("wait_at_waypoint", wait_at_waypoint_);
    get_parameter("save_dir", save_dir_as_string);
    get_parameter("image_format", image_format_);
    get_parameter("waypoint_pause_duration", waypoint_pause_duration_);
    get_parameter("topics", image_topics_);

    // get inputted save directory and make sure it exists, if not log and
    // create  it

    RCLCPP_INFO(logger_, "Params:");

    RCLCPP_INFO(logger_, "trigger_topic: %s", trigger_topic_.c_str());

    RCLCPP_INFO(logger_, "enabled: %d", is_enabled_);

    RCLCPP_INFO(logger_, "wait_at_waypoint: %d", wait_at_waypoint_);

    RCLCPP_INFO(logger_, "save_dir: %s", save_dir_as_string.c_str());

    RCLCPP_INFO(logger_, "image_format: %s", image_format_.c_str());
    RCLCPP_INFO(logger_, "waypoint_pause_duration: %d",
                waypoint_pause_duration_);

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

    callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    for (unsigned i = 0; i < image_topics_.size(); i++) {

      std::filesystem::path dir = save_dir_as_string + image_topics_[i];

      if (!std::filesystem::exists(dir)) {
        RCLCPP_WARN(logger_,
                    "Provided save directory %s, the directory will be created "
                    "automatically.",
                    dir.c_str());
        if (!std::filesystem::create_directory(dir)) {
          RCLCPP_ERROR(logger_,
                       "Failed to create directory!: %s required by "
                       "Turtlebot4_photographer!",
                       dir.c_str());
          continue;
        }
      }

      subscriptions_.push_back(
          create_image_subscription(image_topics_[i], sub_options));
      curr_frame_msgs_.push_back(std::make_shared<sensor_msgs::msg::Image>());
      save_dirs_.push_back(dir);
      image_mutexes_.push_back(std::unique_ptr<std::mutex>());
      topic_to_idx_.insert({image_topics_[i], i});
    }

    trigger_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        trigger_topic_, 1,
        std::bind(&Photographer::triggerCallback, this, std::placeholders::_1),
        sub_options);

    done_publisher_ = create_publisher<std_msgs::msg::Empty>(
        "/input_output_at_waypoint/input", 10);
  }

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>>
  create_image_subscription(const std::string &topic_name,
                            rclcpp::SubscriptionOptions sub_options) {
    auto subscription = create_subscription<sensor_msgs::msg::Image>(
        topic_name, rclcpp::SystemDefaultsQoS(),
        [this, topic_name](sensor_msgs::msg::Image::SharedPtr msg) {
          unsigned idx = topic_to_idx_.find(topic_name)->second;
          std::lock_guard<std::mutex> guard(*image_mutexes_[idx]);
          curr_frame_msgs_[idx] = msg;
        },
        sub_options);
    return subscription;
  }

  void triggerCallback(const geometry_msgs::msg::PoseStamped &curr_pose) {

    RCLCPP_INFO(logger_, "Arrived at waypoint at pos (%f, %f)",
                curr_pose.pose.position.x, curr_pose.pose.position.y);

    if (wait_at_waypoint_) {
      RCLCPP_INFO(logger_, "Sleeping for %i ms before saving picture..",
                  waypoint_pause_duration_);

      clock_->sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
    }

    std::array<double, 3> euler;

    quat_to_euler(curr_pose.pose.orientation, euler);

    for (unsigned i = 0; i < image_topics_.size(); i++) {

      try {

        std::filesystem::path file_name =
            curr_frame_msgs_[i]->header.frame_id + "+" +
            std::to_string(curr_pose.pose.position.x) + "+" +
            std::to_string(curr_pose.pose.position.y) + "+" +
            std::to_string(euler[2]) + "." + image_format_;

        std::filesystem::path full_path = save_dirs_[i] / file_name;
        std::lock_guard<std::mutex> guard(*image_mutexes_[i]);

        cv::Mat curr_frame_mat;
        deepCopyMsg2Mat(curr_frame_msgs_[i], curr_frame_mat);
        cv::imwrite(full_path.c_str(), curr_frame_mat);

      } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_,
                     "Couldn't take photo at waypoint! Caught exception: %s "
                     "with topic %s",
                     e.what(), image_topics_[i].c_str());
      }

      RCLCPP_INFO(logger_, "Photos have been taken sucessfully at waypoint");

      std_msgs::msg::Empty done_trigger;

      done_publisher_->publish(done_trigger);
    }
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

  std::vector<std::string> image_topics_;

  std::unordered_map<std::string, unsigned> topic_to_idx_;

  std::vector<std::filesystem::path> save_dirs_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
      subscriptions_;

  std::vector<sensor_msgs::msg::Image::SharedPtr> curr_frame_msgs_;

  std::vector<std::unique_ptr<std::mutex>> image_mutexes_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      trigger_subscriber_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

} // namespace turtlebot4_photographer

RCLCPP_COMPONENTS_REGISTER_NODE(turtlebot4_photographer::Photographer)

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Photographer>());
//   rclcpp::shutdown();
//   return 0;
// }