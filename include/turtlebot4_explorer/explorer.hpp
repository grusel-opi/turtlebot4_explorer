#ifndef TURTLEBOT4_EXPLORER_HPP
#define TURTLEBOT4_EXPLORER_HPP


#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "nav_msgs/srv/get_map.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "turtlebot4_explorer/util.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/dock.hpp"


class Explorer : public rclcpp::Node {
public:
    Explorer();

    using NavAction = nav2_msgs::action::NavigateToPose;
    using NavClient = rclcpp_action::Client<NavAction>;

    using UndockAction = irobot_create_msgs::action::Undock;    
    using UndockClient = rclcpp_action::Client<UndockAction>;
    using DockAction = irobot_create_msgs::action::Dock;
    using DockClient = rclcpp_action::Client<DockAction>;

    void start();

    void calculateGridPattern();

private:

    NavClient::SharedPtr pose_navigator_;

    std::shared_future<rclcpp_action::ClientGoalHandle<NavAction>::SharedPtr> future_goal_handle_;

    std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> global_costmap_sub_;

    UndockClient::SharedPtr undock_client_;

    DockClient::SharedPtr dock_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;

    nav2_costmap_2d::Costmap2D costmap_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

    std::vector<Frontier> frontiers_;

    GoalStatus current_goal_status_;
    
    Frontier current_goal_;
    
    visualization_msgs::msg::MarkerArray marker_array;

    double loop_rate_;
    
    std::vector<std::array<double, 4>> aborted_;
        
    std::string map_path_;

    bool is_exploring_;

    bool map_received_;

    int min_free_;

    double min_dist_;

    int min_size_;

    geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose_;

    geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr start_pose_;

    std::array<unsigned char, 256> cost_translation_table_;

    void mapCallback(nav_msgs::msg::OccupancyGrid::UniquePtr occupancy_grid);

    void poseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose);

    void findFrontiers();

    bool isAchievableFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag);

    Frontier buildNewFrontier(unsigned int neighbor_cell, std::vector<bool> &frontier_flag, geometry_msgs::msg::Point robot_position);

    void explore();

    void drawMarkers(const std::vector<Frontier> &frontiers);

    void clearMarkers();

    void stop();

    void saveMap();

    void navigationResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    void navigationFeedbackCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> &feedback);

    void navigationResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle);

    bool checkGoal();
};

#endif
