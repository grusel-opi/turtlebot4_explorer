#include <array>
#include <cstddef>
#include <stack>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "tf2_ros/buffer.h"
#include "turtlebot4_explorer/util.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <slam_toolbox/srv/detail/save_map__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "turtlebot4_explorer/util.hpp"

using namespace std::chrono_literals;

using NavAction = nav2_msgs::action::NavigateToPose;
using NavClient = rclcpp_action::Client<NavAction>;
using WaypointAction = nav2_msgs::action::FollowWaypoints;
using WaypointClient = rclcpp_action::Client<WaypointAction>;

class Explorer : public rclcpp::Node {
public:
  Explorer() : Node("Turtlebot4_explorer") {
    RCLCPP_INFO(get_logger(), "Turtlebot4 explorer startup.");

    declare_parameter("map_path", rclcpp::ParameterValue(std::string("~")));
    declare_parameter("min_size", rclcpp::ParameterValue(5));
    declare_parameter("min_dist", rclcpp::ParameterValue(1.0));
    declare_parameter("map_given", rclcpp::ParameterValue(false));
    declare_parameter("lower_cost_bound", rclcpp::ParameterValue(0));
    declare_parameter("upper_cost_bound", rclcpp::ParameterValue(200));
    declare_parameter("coverage_pose_distance", rclcpp::ParameterValue(0.5));
    declare_parameter("coverage_orientations",
                      rclcpp::ParameterValue(std::vector<double>{
                          0., 45., 90., 135., 180., 225., 270., 315.}));

    get_parameter("map_path", map_path_);
    get_parameter("min_size", min_size_);
    get_parameter("min_dist", min_dist_);
    get_parameter("map_given", map_given_);
    get_parameter("lower_cost_bound", lower_cost_bound_);
    get_parameter("upper_cost_bound", upper_cost_bound_);
    get_parameter("coverage_pose_distance", coverage_step_size_w_);
    get_parameter("coverage_orientations", coverage_orientations_deg_);

    RCLCPP_INFO(get_logger(), "map_path: %s", map_path_.c_str());
    RCLCPP_INFO(get_logger(), "min_size: %d", min_size_);
    RCLCPP_INFO(get_logger(), "min_dist: %f", min_dist_);
    RCLCPP_INFO(get_logger(), "map_given: %d", map_given_);
    RCLCPP_INFO(get_logger(), "lower_cost_bound: %d", lower_cost_bound_);
    RCLCPP_INFO(get_logger(), "upper_cost_bound: %d", upper_cost_bound_);
    RCLCPP_INFO(get_logger(), "coverage_pose_distance: %f",
                coverage_step_size_w_);
    RCLCPP_INFO(get_logger(), "number of orientations per coverage pose: %ld",
                coverage_orientations_deg_.size());

    if (map_given_) {
      pose_topic_ = "/amcl_pose";
    } else {
      pose_topic_ = "/pose";
    }

    pose_navigator_ =
        rclcpp_action::create_client<NavAction>(this, "/navigate_to_pose");

    waypoint_navigator_ =
        rclcpp_action::create_client<WaypointAction>(this, "/follow_waypoints");

    pose_subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, 10,
            std::bind(&Explorer::poseCallback, this, std::placeholders::_1));

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&Explorer::mapCallback, this, std::placeholders::_1));

    marker_array_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers",
                                                               10);

    cost_translation_table_ = initTranslationTable();

    start_pose_ =
        std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

    callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    get_global_costmap_client_ = create_client<nav2_msgs::srv::GetCostmap>(
        "/global_costmap/get_costmap", rmw_qos_profile_services_default,
        callback_group_);

    RCLCPP_INFO(get_logger(), "Creating wall timer for start routine..");
    start_timer_ = this->create_wall_timer(2s, [this]() { start(); });
  }

  void start() {

    if (!current_pose_) {
      RCLCPP_INFO(get_logger(), "No pose message yet..");
      return;
    }
    this->start_timer_->cancel();

    RCLCPP_INFO(get_logger(), "Waiting for nav2 stack..");
    pose_navigator_->wait_for_action_server();
    waypoint_navigator_->wait_for_action_server();

    RCLCPP_INFO(get_logger(), "Remembering start pose at (%f, %f), ",
                current_pose_->pose.pose.position.x,
                current_pose_->pose.pose.position.y);
    start_pose_->pose.pose.position = current_pose_->pose.pose.position;
    start_pose_->pose.pose.orientation = current_pose_->pose.pose.orientation;

    if (map_given_) {
      RCLCPP_INFO(get_logger(), "Starting with coverage.");
      calculateCoverage();
    } else {
      RCLCPP_INFO(get_logger(), "Starting with exploration.");
      explore();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr start_timer_;

  NavClient::SharedPtr pose_navigator_;
  WaypointClient::SharedPtr waypoint_navigator_;

  std::shared_future<rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr>
      nav_action_future_goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<WaypointAction>::SharedPtr>
      waypoint_future_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_subscription_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      map_subscription_;

  nav2_costmap_2d::Costmap2D costmap_;

  std::vector<Frontier> frontiers_;
  std::vector<std::array<double, 4>> aborted_;
  Frontier current_goal_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;
  visualization_msgs::msg::MarkerArray marker_array_;

  bool map_given_;
  std::string map_path_;
  std::string pose_topic_;
  double min_dist_;
  unsigned int min_size_;
  unsigned int upper_cost_bound_;
  unsigned int lower_cost_bound_;

  int current_coverage_pose_nr_ = 0;
  double coverage_step_size_w_; // min step size in world scale to execute
                                // coverage pattern
  std::vector<geometry_msgs::msg::Point> coverage_positions_sorted_;
  std::vector<double> coverage_orientations_deg_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoint_poses_;

  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr current_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr start_pose_;

  std::array<unsigned char, 256> cost_translation_table_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr
      get_global_costmap_client_;

  void explore() {

    findFrontiers();

    if (frontiers_.size() == 0) {
      RCLCPP_WARN(get_logger(), "No frontier found!");
      stopExploration();
      return;
    }

    auto goal = NavAction::Goal();
    goal.pose.pose.position = frontiers_[0].centroid;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = "map";

    auto send_goal_options = NavClient::SendGoalOptions();
    // send_goal_options.feedback_callback =
    // std::bind(&Explorer::navigationFeedbackCallback, this,
    // std::placeholders::_1, std::placeholders::_2);
    send_goal_options.goal_response_callback = std::bind(
        &Explorer::navigationResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(
        &Explorer::navigationResultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontiers_[0].centroid.x,
                frontiers_[0].centroid.y);
    nav_action_future_goal_handle_ =
        pose_navigator_->async_send_goal(goal, send_goal_options);
    current_goal_ = frontiers_[0];
  }

  bool checkGoal() {

    unsigned int mx, my, free_count = 0;
    unsigned char *costmap_data = costmap_.getCharMap();
    const int thresh = 5;

    for (const auto &p : current_goal_.points) {
      if (!costmap_.worldToMap(p.x, p.y, mx, my)) {
        RCLCPP_ERROR(get_logger(), "Should not happen!");
        return false;
      }
      unsigned int pos = costmap_.getIndex(mx, my);
      if (costmap_data[pos] == nav2_costmap_2d::NO_INFORMATION) {
        if (++free_count > thresh) {
          return true;
        }
      }
    }
    return false;
  }

  void findFrontiers() {

    frontiers_.clear();

    const auto position = current_pose_->pose.pose.position;
    unsigned int mx, my;

    if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
      RCLCPP_ERROR(get_logger(),
                   "Robot out of costmap bounds, cannot search for frontiers");
      return;
    }

    RCLCPP_INFO(get_logger(), "[findFrontiers] current position %f, %f",
                position.x, position.y);

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap_.getMutex()));

    auto map = costmap_.getCharMap();

    std::vector<bool> frontier_flag(
        costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY(), false);
    std::vector<bool> visited_flag(
        costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY(), false);

    std::queue<unsigned int> bfs;

    unsigned int pos = costmap_.getIndex(mx, my);
    unsigned int start;

    if (nearestCell(start, pos, nav2_costmap_2d::FREE_SPACE, costmap_)) {
      bfs.push(start);
    } else {
      bfs.push(pos);
      RCLCPP_ERROR(get_logger(),
                   "Could not find nearby clear cell to start search");
    }

    visited_flag[bfs.front()] = true;

    while (!bfs.empty()) {

      unsigned int idx = bfs.front();
      bfs.pop();

      for (unsigned nbr : nhood4(idx, costmap_)) {

        if (map[nbr] <= map[idx] && !visited_flag[nbr]) {
          visited_flag[nbr] = true;
          bfs.push(nbr);

        } else if (isAchievableFrontierCell(nbr, frontier_flag)) {
          frontier_flag[nbr] = true;

          Frontier frontier = buildNewFrontier(nbr, frontier_flag, position);
          bool aborted = false;
          for (const auto &bb : aborted_) {
            if (pointInBB(bb, frontier.centroid)) {
              RCLCPP_ERROR(get_logger(),
                           "abort check: (%f, %f) is in area %f - %f, %f - %f",
                           frontier.centroid.x, frontier.centroid.y, bb[0],
                           bb[1], bb[2], bb[3]);
              aborted = true;
              break;
            }
          }
          if (!aborted && frontier.distance > min_dist_ &&
              frontier.points.size() > min_size_) {
            frontiers_.push_back(frontier);
          }
        }
      }
    }

    std::sort(frontiers_.begin(), frontiers_.end(), compareFrontiers);

    drawMarkers(frontiers_);
  }

  Frontier buildNewFrontier(unsigned int neighborCell,
                            std::vector<bool> &frontier_flag,
                            geometry_msgs::msg::Point robot_position) {

    Frontier output;

    frontier_flag[neighborCell] = true;

    unsigned int mx, my;
    double wx, wy;
    costmap_.indexToCells(neighborCell, mx, my);
    costmap_.mapToWorld(mx, my, wx, wy);

    geometry_msgs::msg::Point point;
    point.x = wx;
    point.y = wy;

    output.points.push_back(point);

    output.centroid.x += wx;
    output.centroid.y += wy;

    std::queue<unsigned int> bfs;
    bfs.push(neighborCell);

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();

      for (unsigned int nbr : nhood8(idx, costmap_)) {

        if (isAchievableFrontierCell(nbr, frontier_flag)) {

          frontier_flag[nbr] = true;

          costmap_.indexToCells(nbr, mx, my);
          costmap_.mapToWorld(mx, my, wx, wy);

          geometry_msgs::msg::Point point;
          point.x = wx;
          point.y = wy;

          output.points.push_back(point);

          output.centroid.x += wx;
          output.centroid.y += wy;

          bfs.push(nbr);
        }
      }
    }

    output.centroid.x /= output.points.size();
    output.centroid.y /= output.points.size();

    double dx = robot_position.x - output.centroid.x;
    double dy = robot_position.y - output.centroid.y;

    // double dx = currentGoal.centroid.x - output.centroid.x;
    // double dy = currentGoal.centroid.y - output.centroid.y;

    output.distance = std::sqrt(dx * dx + dy * dy);

    return output;
  }

  bool isAchievableFrontierCell(unsigned int idx,
                                const std::vector<bool> &frontier_flag) {

    auto map = costmap_.getCharMap();

    if (map[idx] != nav2_costmap_2d::NO_INFORMATION || frontier_flag[idx]) {
      return false;
    }

    for (unsigned int nbr : nhood4(idx, costmap_)) {
      if (map[nbr] < 200) { // TODO: make this thresh configurable?
        return true;
      }
    }

    return false;
  }

  void navigationResponseCallback(
      const rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr
          &goal_handle) {
    if (goal_handle) {
      RCLCPP_INFO(get_logger(), "[RESPONSE] Goal accepted by server.");
    } else {
      RCLCPP_ERROR(get_logger(), "[RESPONSE] Goal was rejected by server.");
      explore();
    }
  }

  void navigationFeedbackCallback(
      const rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr &,
      const std::shared_ptr<const NavAction::Feedback> &feedback) {
    RCLCPP_INFO(get_logger(), "Distance remaining: %f",
                feedback->distance_remaining);
  }

  void navigationResultCallback(
      const rclcpp_action::ClientGoalHandle<NavAction>::WrappedResult &result) {
    if (result.goal_id != nav_action_future_goal_handle_.get()->get_goal_id()) {
      RCLCPP_DEBUG(get_logger(),
                   "[RESULT] Goal IDs do not match for the current goal handle "
                   "and received result."
                   "Ignoring likely due to receiving result for an old goal.");
      return;
    }

    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(),
                  "[RESULT] Goal result: reached. Still unexplored: %d",
                  checkGoal());
      break;
    case rclcpp_action::ResultCode::ABORTED: {
      auto bb = frontierToBB(current_goal_, costmap_.getResolution());
      RCLCPP_ERROR(get_logger(),
                   "[RESULT] Goal result: aborted, marking as unreachable: "
                   "x:%f-%f, y: %f-%f Still unexplored: %d",
                   bb[0], bb[1], bb[2], bb[3], checkGoal());
      aborted_.push_back(bb);
    } break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(),
                   "[RESULT] Goal result: canceled. Still unexplored: %d",
                   checkGoal());
      break;
    default:
      RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: Unknown");
      break;
    }
    explore();
  }

  void mapCallback(nav_msgs::msg::OccupancyGrid::UniquePtr occupancyGrid) {

    const auto occupancyGridInfo = occupancyGrid->info;
    costmap_.resizeMap(occupancyGridInfo.width, occupancyGridInfo.height,
                       occupancyGridInfo.resolution,
                       occupancyGridInfo.origin.position.x,
                       occupancyGridInfo.origin.position.y);

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *costmap_.getMutex());

    unsigned char *costmap_data = costmap_.getCharMap();
    size_t costmap_size =
        costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
    for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size();
         ++i) {
      auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
      costmap_data[i] = cost_translation_table_[cell_cost];
    }
  }

  void poseCallback(
      geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr poseMsg) {
    // RCLCPP_INFO(get_logger(), "poseCallback..");
    current_pose_ = move(poseMsg);
  }

  void drawMarkers(const std::vector<Frontier> &frontiers) {

    clearMarkers();

    for (unsigned int i = 0; i < frontiers.size(); i++) {

      const Frontier &frontier = frontiers[i];

      std_msgs::msg::ColorRGBA color;

      if (i == 0) {
        color.r = 1.0;
        color.g = 0;
        color.b = 0;
        color.a = 1.0;
      } else {
        color.r = 0;
        color.g = 1.0;
        color.b = 0;
        color.a = 1.0;
      }

      std::vector<visualization_msgs::msg::Marker> &markers =
          marker_array_.markers;
      visualization_msgs::msg::Marker m;

      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.frame_locked = true;

      m.action = visualization_msgs::msg::Marker::ADD;
      m.ns = "frontiers";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.pose.position = frontier.centroid;
      m.scale.x = 0.3;
      m.scale.y = 0.3;
      m.scale.z = 0.3;
      m.color = color;
      markers.push_back(m);
      marker_array_publisher_->publish(marker_array_);
    }
  }

  void clearMarkers() {

    for (auto &m : marker_array_.markers) {
      m.action = visualization_msgs::msg::Marker::DELETE;
    }
    marker_array_publisher_->publish(marker_array_);
    marker_array_.markers.clear();
  }

  void stopExploration() {
    RCLCPP_INFO(get_logger(), "Explorer end. Returning to start pose.");

    auto goal = NavAction::Goal();
    goal.pose.pose.position = start_pose_->pose.pose.position;
    goal.pose.pose.orientation = start_pose_->pose.pose.orientation;
    goal.behavior_tree =
        "/home/mayerfel/ws/autonomous_acquisition/bt/nav2home.xml";
    goal.pose.header.frame_id = "map";

    auto send_goal_options = NavClient::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const auto msg) {
      if (msg) {
        RCLCPP_INFO(get_logger(), "Return to start goal accepted by server.");
      } else {
        RCLCPP_ERROR(get_logger(),
                     "Return to start goal was rejected by server.");
      }
    };

    send_goal_options.result_callback = [this](const auto result) {
      switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Return to start goal result: reached.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Return to start goal result: aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Return to start goal result: canceled.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Return to start goal result: Unknown");
        break;
      }
      pose_subscription_.reset();
      map_subscription_.reset();
      pose_navigator_->async_cancel_all_goals();

      saveMap();
      clearMarkers();
      calculateCoverage();
    };

    RCLCPP_INFO(get_logger(), "Sending return to home goal %f,%f",
                goal.pose.pose.position.x, goal.pose.pose.position.y);
    auto return_goal_handle =
        pose_navigator_->async_send_goal(goal, send_goal_options);
  }

  void saveMap() {
    auto mapSerializer = create_client<slam_toolbox::srv::SerializePoseGraph>(
        "/slam_toolbox/serialize_map");
    auto serializePoseGraphRequest =
        std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();

    serializePoseGraphRequest->filename = map_path_;
    RCLCPP_INFO(get_logger(), "Sending request to /slam_toolbox/serialize_map");
    auto serializePoseResult =
        mapSerializer->async_send_request(serializePoseGraphRequest);

    auto map_saver =
        create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");
    auto saveMapRequest =
        std::make_shared<slam_toolbox::srv::SaveMap::Request>();
    saveMapRequest->name.data = map_path_;
    RCLCPP_INFO(get_logger(), "Sending request to /slam_toolbox/save_map");
    auto saveMapResult = map_saver->async_send_request(saveMapRequest);
  }

  void calculateCoverage() {

    std::vector<geometry_msgs::msg::Point> positions;
    coverage_positions_sorted_.clear();

    if (!costmapWalkSampling(positions)) {
      RCLCPP_ERROR(get_logger(), "Error calculating coverage poses!");
      return;
    }

    cheapTSP(positions);

    // simpleBoustrophedonOrdering();
    backAndForthOrdering();

    // executeStarPatternCoverageViaWaypoints();
    executeCoverage();
  }

  bool costmapWalkSampling(std::vector<geometry_msgs::msg::Point> &positions) {

    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();

    RCLCPP_INFO(get_logger(),
                "waiting for sercive /global_costmap/get_costmap");
    while (!get_global_costmap_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(get_logger(), "service not available, waiting again 1s...");
    }

    RCLCPP_INFO(get_logger(), "sending request to /global_costmap/get_costmap");

    auto result = get_global_costmap_client_->async_send_request(request);

    auto status = result.wait_for(5s); // 5s should be generous
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Houston weve had a problem!");
      return false;
    }

    RCLCPP_INFO(get_logger(), "success!");

    nav2_costmap_2d::Costmap2D costmap;
    nav2_msgs::msg::Costmap map;

    map = result.get()->map;

    const auto meta_data = map.metadata;
    costmap.resizeMap(meta_data.size_x, meta_data.size_y, meta_data.resolution,
                      meta_data.origin.position.x, meta_data.origin.position.y);

    unsigned char *costmap_data = costmap.getCharMap();
    size_t costmap_size = costmap.getSizeInCellsX() * costmap.getSizeInCellsY();
    for (size_t i = 0; i < costmap_size && i < map.data.size(); ++i) {
      costmap_data[i] = map.data[i];
    }

    const auto position = start_pose_->pose.pose.position;
    unsigned int mx, my;

    if (!costmap.worldToMap(position.x, position.y, mx, my)) {
      RCLCPP_ERROR(get_logger(), "Robot start position out of costmap bounds, "
                                 "should not be possible..");
      return false;
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap.getMutex()));

    auto char_map = costmap.getCharMap();

    std::vector<bool> frontier_flag(
        costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);
    std::vector<bool> visited_flag(
        costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);

    std::stack<unsigned int> dfs;

    unsigned char cost = costmap.getCost(mx, my);
    unsigned int pos_idx = costmap.getIndex(mx, my);

    RCLCPP_INFO(get_logger(), "start cost: %u", cost);

    while (cost > upper_cost_bound_ || cost < lower_cost_bound_) {
      for (unsigned nbr : nhood8(pos_idx, costmap)) {
        if ((cost > upper_cost_bound_ && char_map[nbr] <= cost) ||
            (cost < lower_cost_bound_ && char_map[nbr] >= cost)) {
          cost = char_map[nbr];
          pos_idx = nbr;
        }
      }
    }

    RCLCPP_INFO(get_logger(), "new cost: %u", cost);

    dfs.push(pos_idx);

    geometry_msgs::msg::Point pos;
    costmap.indexToCells(pos_idx, mx, my);
    costmap.mapToWorld(mx, my, pos.x, pos.y);

    visited_flag[dfs.top()] = true;

    while (!dfs.empty()) {

      unsigned int idx = dfs.top();
      dfs.pop();

      if (char_map[idx] <= upper_cost_bound_ &&
          char_map[idx] >= lower_cost_bound_) {
        costmap.indexToCells(idx, mx, my);
        costmap.mapToWorld(mx, my, pos.x, pos.y);
        positions.push_back(pos);
      }

      for (unsigned nbr : nhood8(idx, costmap)) {

        if (!visited_flag[nbr] && char_map[nbr] <= upper_cost_bound_) {
          visited_flag[nbr] = true;
          dfs.push(nbr);
        }
      }
    }
    return true;
  }

  void executeStarPatternCoverageViaWaypoints() {

    for (const auto &p : coverage_positions_sorted_) {
      for (const auto &o : coverage_orientations_deg_) {
        geometry_msgs::msg::PoseStamped waypoint_pose;
        waypoint_pose.pose.position = p;
        waypoint_pose.pose.orientation.z =
            std::sin((o / 360. * (2 * M_PI)) / 2);
        waypoint_pose.pose.orientation.w =
            std::cos((o / 360. * (2 * M_PI)) / 2);
        waypoint_pose.header.frame_id = "map";

        waypoint_poses_.push_back(waypoint_pose);
      }
    }

    int max = waypoint_poses_.size();
    for (unsigned int i = 0; i < max; i++) {

      std_msgs::msg::ColorRGBA color;

      color.r = ((double)max - (double)i) / (double)max;
      color.g = 0.;
      color.b = (double)i / (double)max;
      color.a = 1.0;

      std::vector<visualization_msgs::msg::Marker> &markers =
          marker_array_.markers;
      visualization_msgs::msg::Marker m;

      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.frame_locked = true;

      m.action = visualization_msgs::msg::Marker::ADD;
      m.ns = "grid_pattern";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.pose = waypoint_poses_[i].pose;
      m.scale.x = 0.15;
      m.scale.y = 0.05;
      m.scale.z = 0.05;
      m.color = color;
      markers.push_back(m);
    }
    marker_array_publisher_->publish(marker_array_);
    RCLCPP_INFO(get_logger(), "published poses number: %ld",
                marker_array_.markers.size());

    auto goal = WaypointAction::Goal();
    goal.poses = waypoint_poses_;

    auto send_goal_options = WaypointClient::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const auto &goal_handle) {
      if (goal_handle) {
        RCLCPP_INFO(get_logger(),
                    "[RESPONSE] Waypoint goal accepted by server.");
      } else {
        RCLCPP_ERROR(get_logger(),
                     "[RESPONSE] Waypoint goal was rejected by server.");
      }
    };

    send_goal_options.result_callback = [this](const auto &result) {
      if (result.goal_id != waypoint_future_goal_handle_.get()->get_goal_id()) {
        RCLCPP_DEBUG(get_logger(), "[RESULT] Goal IDs do not match for the "
                                   "current goal handle and received result."
                                   "This is unlikely using the waypoint "
                                   "navigator. There is something wrong!");
      }

      switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "[RESULT] Goal result: SUCCEEDED.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: canceled.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: Unknown");
        break;
      }

      RCLCPP_INFO(get_logger(), "[RESULT] Missed waypoint poses:");
      for (const int idx : result.result->missed_waypoints) {
        std::array<double, 3> euler;
        quat_to_euler(waypoint_poses_[idx].pose.orientation, euler);
        RCLCPP_INFO(get_logger(), "x: %f, y: %f, yaw: %f",
                    waypoint_poses_[idx].pose.position.x,
                    waypoint_poses_[idx].pose.position.y, euler[2]);
      }
      returnToStartPose();
    };

    RCLCPP_INFO(get_logger(), "[REQUEST] Sending goals to waypoint navigator.");
    waypoint_future_goal_handle_ =
        waypoint_navigator_->async_send_goal(goal, send_goal_options);
  }

  void returnToStartPose() {
    RCLCPP_INFO(get_logger(), "Returning to start position.");

    auto goal = NavAction::Goal();
    goal.pose.pose.position = start_pose_->pose.pose.position;
    goal.pose.pose.orientation = start_pose_->pose.pose.orientation;
    goal.behavior_tree =
        "/home/mayerfel/ws/autonomous_acquisition/bt/nav2home.xml";
    goal.pose.header.frame_id = "map";

    auto send_goal_options = NavClient::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const auto msg) {
      if (msg) {
        RCLCPP_INFO(get_logger(), "Return to start goal accepted by server.");
      } else {
        RCLCPP_ERROR(get_logger(),
                     "Return to start goal was rejected by server.");
      }
    };

    send_goal_options.result_callback = [this](const auto result) {
      switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Return to start goal result: reached.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Return to start goal result: aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Return to start goal result: canceled.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Return to start goal result: Unknown");
        break;
      }
    };

    RCLCPP_INFO(get_logger(), "Sending return to home goal %f,%f",
                goal.pose.pose.position.x, goal.pose.pose.position.y);
    auto return_goal_handle =
        pose_navigator_->async_send_goal(goal, send_goal_options);
  }

  void executeCoverage() {

    RCLCPP_INFO(get_logger(), "[executeCoverage]");

    geometry_msgs::msg::Point next_goal =
        coverage_positions_sorted_[current_coverage_pose_nr_];

    auto goal = NavAction::Goal();
    goal.pose.pose.position = next_goal;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = "map";

    auto send_goal_options = NavClient::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const auto &goal_handle) {
      if (goal_handle) {
        RCLCPP_INFO(get_logger(), "[RESPONSE] Goal accepted by server.");
      } else {
        RCLCPP_ERROR(get_logger(), "[RESPONSE] Goal was rejected by server.");
        current_coverage_pose_nr_++;
        if (current_coverage_pose_nr_ <
                (int)coverage_positions_sorted_.size() &&
            current_coverage_pose_nr_ >= 0) {
          executeCoverage();
        } else {
          RCLCPP_INFO(get_logger(),
                      "[executeCoverage] Seems like coverage is completed! "
                      "current_coverage_pose_nr_: %d",
                      current_coverage_pose_nr_);
        }
      }
    };

    send_goal_options.result_callback = [this](const auto &result) {
      if (result.goal_id !=
          nav_action_future_goal_handle_.get()->get_goal_id()) {
        RCLCPP_DEBUG(
            get_logger(),
            "[RESULT] Goal IDs do not match for the current goal handle and "
            "received result."
            "Ignoring likely due to receiving result for an old goal.");
        return;
      }

      switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "[RESULT] Goal result: reached.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(),
                     "[RESULT] Goal result: aborted, continuing with next..");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: canceled.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: Unknown");
        break;
      }
      current_coverage_pose_nr_++;
      if (current_coverage_pose_nr_ < (int)coverage_positions_sorted_.size() &&
          current_coverage_pose_nr_ >= 0) {
        executeCoverage();
      } else {
        RCLCPP_INFO(get_logger(),
                    "[executeCoverage] Seems like coverage is completed! "
                    "current_coverage_pose_nr_: %d",
                    current_coverage_pose_nr_);
        returnToStartPose();
      }
    };

    RCLCPP_INFO(get_logger(), "[REQUEST] Sending goal %f,%f", next_goal.x,
                next_goal.y);
    nav_action_future_goal_handle_ =
        pose_navigator_->async_send_goal(goal, send_goal_options);
  }

  void backAndForthOrdering() {

    RCLCPP_INFO(get_logger(), "[backAndForthOrdering]");

    std::vector<geometry_msgs::msg::Point> back_and_forth;
    unsigned int n = coverage_positions_sorted_.size();

    for (unsigned i = 0; i < n; i++) {
      back_and_forth.push_back(coverage_positions_sorted_[i]);
    }
    for (unsigned i = n - 1; i > 0; i--) {
      back_and_forth.push_back(coverage_positions_sorted_[i]);
    }

    coverage_positions_sorted_ = back_and_forth;

    int max = coverage_positions_sorted_.size();
    for (unsigned int i = 0; i < max; i++) {

      std_msgs::msg::ColorRGBA color;

      color.r = ((double)max - (double)i) / (double)max;
      color.g = 0.;
      color.b = (double)i / (double)max;
      color.a = 1.0;

      std::vector<visualization_msgs::msg::Marker> &markers =
          marker_array_.markers;
      visualization_msgs::msg::Marker m;

      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.frame_locked = true;

      m.action = visualization_msgs::msg::Marker::ADD;
      m.ns = "grid_pattern";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.pose.position = coverage_positions_sorted_[i];
      m.scale.x = 0.15;
      m.scale.y = 0.05;
      m.scale.z = 0.05;
      m.color = color;
      markers.push_back(m);
    }
    marker_array_publisher_->publish(marker_array_);
    RCLCPP_INFO(get_logger(), "published poses number: %ld",
                marker_array_.markers.size());
  }

  void simpleBoustrophedonOrdering() {

    std::vector<geometry_msgs::msg::Point> boustrophedon_coverage_points;
    unsigned int n = coverage_positions_sorted_.size();

    for (unsigned i = 0; i < n / 2; i++) {
      boustrophedon_coverage_points.push_back(coverage_positions_sorted_[i]);
      boustrophedon_coverage_points.push_back(
          coverage_positions_sorted_[n - i]);
    }
    coverage_positions_sorted_ = boustrophedon_coverage_points;

    int max = coverage_positions_sorted_.size();
    for (unsigned int i = 0; i < max; i++) {

      std_msgs::msg::ColorRGBA color;

      color.r = ((double)max - (double)i) / (double)max;
      color.g = 0.;
      color.b = (double)i / (double)max;
      color.a = 1.0;

      std::vector<visualization_msgs::msg::Marker> &markers =
          marker_array_.markers;
      visualization_msgs::msg::Marker m;

      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.frame_locked = true;

      m.action = visualization_msgs::msg::Marker::ADD;
      m.ns = "grid_pattern";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.pose.position = coverage_positions_sorted_[i];
      m.scale.x = 0.15;
      m.scale.y = 0.05;
      m.scale.z = 0.05;
      m.color = color;
      markers.push_back(m);
    }
    marker_array_publisher_->publish(marker_array_);
    RCLCPP_INFO(get_logger(), "published poses number: %ld",
                marker_array_.markers.size());
  }

  void cheapTSP(std::vector<geometry_msgs::msg::Point> &positions) {

    RCLCPP_INFO(get_logger(), "[cheapTSP]");

    geometry_msgs::msg::Point current_pos = current_pose_->pose.pose.position;

    int best_next_idx = 0;
    double best_dist = 10e10f;

    int todo = positions.size();
    std::vector<bool> planned(todo, false);
    int done = 0;

    while (done < todo) {

      for (int i = 0; i < todo; i++) {
        if (!planned[i]) {
          double tmp_dist =
              std::sqrt(std::pow(current_pos.x - positions[i].x, 2) +
                        std::pow(current_pos.y - positions[i].y, 2));
          if (tmp_dist < best_dist) {
            best_dist = tmp_dist;
            best_next_idx = i;
          }
        }
      }

      if (best_dist >= coverage_step_size_w_) {
        coverage_positions_sorted_.push_back(positions[best_next_idx]);
        current_pos = positions[best_next_idx];
      }

      done++;
      planned[best_next_idx] = true;
      best_dist = 10e10f;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto explorer = std::make_shared<Explorer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(explorer);
  executor.spin();
  return 0;
}