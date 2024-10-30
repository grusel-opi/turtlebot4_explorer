#ifndef TURTLEBOT4_EXPLORER_HPP
#define TURTLEBOT4_EXPLORER_HPP


#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
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

private:

    nav2_costmap_2d::Costmap2D costmap;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr poseNavigator;

    rclcpp_action::Client<irobot_create_msgs::action::Undock>::SharedPtr undockClient;
    rclcpp_action::Client<irobot_create_msgs::action::Dock>::SharedPtr dockClient;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseSubscription;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscription;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerArrayPublisher;

    std::vector<Frontier> frontiers_;

    tf2_ros::Buffer tfBuffer;
    
    tf2_ros::TransformListener tfListener;

    geometry_msgs::msg::TransformStamped laser_to_map_transform;

    geometry_msgs::msg::PoseStamped laser_goal_pose;
    
    visualization_msgs::msg::MarkerArray markerArray;

    rclcpp::TimerBase::SharedPtr timer;
    
    bool isExploring = false;

    std::array<double, 4> currentGoalArea;

    std::vector<std::array<double, 4>> aborted;
        
    std::string map_path;

    bool is_exploring;

    int min_free;

    double min_dist;

    int min_size;

    geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose;

    std::array<unsigned char, 256> costTranslationTable = initTranslationTable();

    void start(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>::WrappedResult &result);

    void sendLaserGoal();

    void scanCallback(sensor_msgs::msg::LaserScan::UniquePtr scan);

    void mapCallback(nav_msgs::msg::OccupancyGrid::UniquePtr occupancyGrid);

    void poseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr pose);

    void findFrontiers();

    bool isAchievableFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag);

    Frontier buildNewFrontier(unsigned int neighborCell, std::vector<bool> &frontier_flag, geometry_msgs::msg::Point robot_position);

    void explore();

    void drawMarkers(const std::vector<Frontier> &frontiers);

    void clearMarkers();

    void stop();

    void saveMap();

    void navigationResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    void navigationFeedbackCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> &feedback);

    void navigationResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle);

    void checkGoal();
};

#endif