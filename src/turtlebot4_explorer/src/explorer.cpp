#include <array>
#include <cstddef>
#include <stack>
#include <vector>

#include "turtlebot4_explorer/util.hpp"

#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <slam_toolbox/srv/detail/save_map__struct.hpp>

using namespace std::chrono_literals;

using NavAction = nav2_msgs::action::NavigateToPose;
using NavClient = rclcpp_action::Client<NavAction>;
using WaypointAction = nav2_msgs::action::FollowWaypoints;
using WaypointClient = rclcpp_action::Client<WaypointAction>;
using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;

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

    pose_subscription_ = create_subscription<PoseCovStamped>(
        pose_topic_, 10,
        std::bind(&Explorer::poseCallback, this, std::placeholders::_1));

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&Explorer::mapCallback, this, std::placeholders::_1));

    marker_array_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers",
                                                               10);

    cost_translation_table_ = initTranslationTable();

    start_pose_ = std::make_unique<PoseCovStamped>();

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

  rclcpp::Subscription<PoseCovStamped>::SharedPtr pose_subscription_;

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

  unsigned long current_coverage_pose_nr_ = 0;
  double coverage_step_size_w_; // min step size in world scale to execute
  // coverage pattern
  std::vector<PoseStamped> coverage_poses_;

  std::vector<double> coverage_orientations_deg_;
  std::vector<PoseStamped> waypoint_poses_;

  PoseCovStamped::UniquePtr current_pose_;
  PoseCovStamped::UniquePtr start_pose_;

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
    goal.behavior_tree =
        "/home/mayerfel/ws/autonomous_acquisition/bt/explore.xml";

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

    if (nearestCell(start, pos, nav2_costmap_2d::FREE_SPACE,
                    nav2_costmap_2d::FREE_SPACE, costmap_)) {
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
                            Point robot_position) {

    Frontier output;

    frontier_flag[neighborCell] = true;

    unsigned int mx, my;
    double wx, wy;
    costmap_.indexToCells(neighborCell, mx, my);
    costmap_.mapToWorld(mx, my, wx, wy);

    Point point;
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

          Point point;
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

  void poseCallback(PoseCovStamped::UniquePtr poseMsg) {
    // RCLCPP_INFO(get_logger(), "poseCallback..");
    current_pose_ = move(poseMsg);
  }

  void drawPositions(std::vector<Point> positions) {

    clearMarkers();

    unsigned max = positions.size();
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
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.pose.position = positions[i];
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.color = color;
      markers.push_back(m);
    }
    marker_array_publisher_->publish(marker_array_);
    RCLCPP_INFO(get_logger(), "published poses number: %ld",
                marker_array_.markers.size());
  }

  void drawPoses(std::vector<PoseStamped> poses) {

    clearMarkers();

    unsigned max = poses.size();
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
      m.pose = poses[i].pose;
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

    std::vector<Point> positions;

    if (!costmapWalkSampling(positions)) {
      RCLCPP_ERROR(get_logger(), "Error calculating coverage poses!");
      return;
    }

    cheapTSP(positions);
    
    worldspacePoseSampling(positions);

    // drawPositions(positions);

    // backAndForthOrdering(positions);

    positionToPathPoses(positions);

    drawPoses(coverage_poses_);

    // executeStarPatternCoverageViaWaypoints(positions);
    executeCoverage();
  }

  bool costmapWalkSampling(std::vector<Point> &positions) {

    RCLCPP_INFO(get_logger(), "[costmapWalkSampling]");

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
      RCLCPP_ERROR(get_logger(), "Did not receive an answer after 5 seconds.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "success!");

    nav2_costmap_2d::Costmap2D costmap;
    nav2_msgs::msg::Costmap map;

    map = result.get()->map;

    costmapMsgTo2D(map, costmap);

    const auto position = current_pose_->pose.pose.position;
    unsigned int mx, my;

    if (!costmap.worldToMap(position.x, position.y, mx, my)) {
      RCLCPP_ERROR(get_logger(), "Robot position out of costmap bounds, "
                                 "should not be possible..");
      return false;
    }

    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(
        costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);

    unsigned int start;
    unsigned int pos_idx = costmap.getIndex(mx, my);
    bfs.push(pos_idx);
    visited_flag[pos_idx] = true;
    unsigned occupied_nbr;

    while (!bfs.empty()) {
      unsigned int idx = bfs.front();
      bfs.pop();

      if (isCostBorderCell(idx, occupied_nbr, costmap)) {
        start = idx;
        break;
      }

      for (unsigned nbr : nhood8(idx, costmap)) {
        if (!visited_flag[nbr]) {
          bfs.push(nbr);
          visited_flag[nbr] = true;
        }
      }
    }

    Point free_pos;
    Point occupied_pos;
    Point coverage_pos;

    std::stack<unsigned int> dfs;
    dfs.push(start);
    visited_flag[dfs.top()] = true;

    while (!dfs.empty()) {

      unsigned int idx = dfs.top();
      dfs.pop();

      if (isCostBorderCell(idx, occupied_nbr, costmap)) {
        costmap.indexToCells(idx, mx, my);
        costmap.mapToWorld(mx, my, free_pos.x, free_pos.y);

        costmap.indexToCells(occupied_nbr, mx, my);
        costmap.mapToWorld(mx, my, occupied_pos.x, occupied_pos.y);

        coverage_pos.x = free_pos.x + (occupied_pos.x - free_pos.x) * 0.4 /
                                          costmap.getResolution();
        coverage_pos.y = free_pos.y + (occupied_pos.y - free_pos.y) * 0.4 /
                                          costmap.getResolution();

        positions.push_back(coverage_pos);
      }

      for (unsigned nbr : nhood8(idx, costmap)) {

        if (!visited_flag[nbr] && costmap.getCost(nbr) < 80 /* && costmap.getCost(nbr) > 0 && isCostBorderCell(nbr, occupied_nbr, costmap)*/) {
          visited_flag[nbr] = true;
          dfs.push(nbr);
        }
      }
    }
    RCLCPP_INFO(get_logger(), "[costmapWalkSampling]: found %lu positions",
                positions.size());

    return true;
  }

  void executeStarPatternCoverageViaWaypoints(std::vector<Point> &positions) {

    for (const auto &p : positions) {
      for (const auto &o : coverage_orientations_deg_) {
        PoseStamped waypoint_pose;
        waypoint_pose.pose.position = p;
        waypoint_pose.pose.orientation.z =
            std::sin((o / 360. * (2 * M_PI)) / 2);
        waypoint_pose.pose.orientation.w =
            std::cos((o / 360. * (2 * M_PI)) / 2);
        waypoint_pose.header.frame_id = "map";

        waypoint_poses_.push_back(waypoint_pose);
      }
    }

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

    RCLCPP_INFO(get_logger(),
                "[executeCoverage]: current_coverage_pose_nr_: %lu",
                current_coverage_pose_nr_);

    PoseStamped next_goal = coverage_poses_[current_coverage_pose_nr_];

    auto goal = NavAction::Goal();
    goal.pose = next_goal;
    goal.behavior_tree =
        "/home/mayerfel/ws/autonomous_acquisition/bt/coverage.xml";

    auto send_goal_options = NavClient::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const auto &goal_handle) {
      if (goal_handle) {
        RCLCPP_INFO(get_logger(), "[RESPONSE] Goal accepted by server.");
      } else {
        RCLCPP_ERROR(get_logger(), "[RESPONSE] Goal was rejected by server.");
        current_coverage_pose_nr_++;
        if (current_coverage_pose_nr_ < coverage_poses_.size()) {
          executeCoverage();
        } else {
          RCLCPP_INFO(get_logger(),
                      "[executeCoverage] Seems like coverage is completed! "
                      "current_coverage_pose_nr_: %lu",
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
      if (current_coverage_pose_nr_ < coverage_poses_.size()) {
        executeCoverage();
      } else {
        RCLCPP_INFO(get_logger(),
                    "[executeCoverage] Seems like coverage is completed! "
                    "current_coverage_pose_nr_: %lu",
                    current_coverage_pose_nr_);
        returnToStartPose();
      }
    };

    RCLCPP_INFO(get_logger(), "[REQUEST] Sending goal %f,%f",
                next_goal.pose.position.x, next_goal.pose.position.y);
    nav_action_future_goal_handle_ =
        pose_navigator_->async_send_goal(goal, send_goal_options);
  }

  void backAndForthOrdering(std::vector<Point> &positions) {

    RCLCPP_INFO(get_logger(), "[backAndForthOrdering]");

    std::vector<Point> back_and_forth;
    unsigned int n = positions.size();

    for (unsigned i = 0; i < n; i++) {
      back_and_forth.push_back(positions[i]);
    }
    for (unsigned i = n - 1; i > 0; i--) {
      back_and_forth.push_back(positions[i]);
    }

    positions = back_and_forth;
  }

  void cheapTSP(std::vector<Point> &positions) {

    RCLCPP_INFO(get_logger(), "[cheapTSP]");

    if (positions.size() == 0) {
      RCLCPP_ERROR(get_logger(), "[cheapTSP] positions empty");
      return;
    }

    Point current_pos = current_pose_->pose.pose.position;

    int best_next_idx = 0;
    double best_dist = 10e10f;

    int todo = positions.size();
    std::vector<Point> positions_sorted(todo);
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
      current_pos = positions[best_next_idx];
      positions_sorted.push_back(positions[best_next_idx]);

      done++;
      planned[best_next_idx] = true;
      best_dist = 10e10f;
    }
    
    positions = positions_sorted;
  }

  void worldspacePoseSampling(std::vector<Point> &positions) {
    std::vector<Point> positions_sampled;

    Point current_pos = positions[0];
    positions_sampled.push_back(current_pos);

    for (unsigned i = 1; i < positions.size(); i++) {
      double tmp_dist = std::sqrt(std::pow(current_pos.x - positions[i].x, 2) +
                                  std::pow(current_pos.y - positions[i].y, 2));
      if (tmp_dist > coverage_step_size_w_) {
        positions_sampled.push_back(positions[i]);
        current_pos = positions[i];
      }
    }

    positions = positions_sampled;

    RCLCPP_INFO(
        get_logger(),
        "[worldspacePoseSampling]: using %lu of %lu positions for coverage",
        positions_sampled.size(), positions.size());
  }

  void positionToPathPoses(std::vector<Point> &positions) {

    RCLCPP_INFO(get_logger(), "[positionToPathPoses]");

    PoseStamped waypoint_pose;
    unsigned i;

    for (i = 0; i < positions.size() - 1; i++) {

      double x = positions[i + 1].x - positions[i].x;
      double y = positions[i + 1].y - positions[i].y;
      double yaw = std::atan2(y, x);

      // RCLCPP_INFO(get_logger(), "[positionToPathPoses] yaw: %f", yaw);

      geometry_msgs::msg::Quaternion q_msg;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      q_msg = tf2::toMsg(q);

      waypoint_pose.pose.position = positions[i];
      waypoint_pose.pose.orientation = q_msg;
      waypoint_pose.header.frame_id = "map";
      coverage_poses_.push_back(waypoint_pose);
    }

    waypoint_pose.pose.position = positions[++i];
    waypoint_pose.pose.orientation.w = 1;
    waypoint_pose.header.frame_id = "map";

    coverage_poses_.push_back(waypoint_pose);
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