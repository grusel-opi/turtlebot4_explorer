#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include <cstddef>
#include <slam_toolbox/srv/detail/save_map__struct.hpp>

#include "turtlebot4_explorer/util.hpp"
#include "turtlebot4_explorer/explorer.hpp"

using namespace std::chrono_literals;


Explorer::Explorer()
: Node("Turtlebot4_explorer"),
  tfBuffer(this->get_clock()),
  tfListener(tfBuffer)
{
    RCLCPP_INFO(get_logger(), "Turtlebot4 explorer startup.");

    declare_parameter("map_path", rclcpp::ParameterValue(std::string("~")));
    declare_parameter("min_dist", rclcpp::ParameterValue(1.0));
    declare_parameter("min_size", rclcpp::ParameterValue(5));

    get_parameter("min_size", min_size);
    get_parameter("min_dist", min_dist);
    get_parameter("map_path", map_path);

    RCLCPP_INFO(get_logger(), "min_size: %d", min_size);
    RCLCPP_INFO(get_logger(), "min_dist: %f", min_dist);
    RCLCPP_INFO(get_logger(), "map_path: %s", map_path.c_str());

    poseSubscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", 10, std::bind(&Explorer::poseCallback, this, std::placeholders::_1));

    mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Explorer::mapCallback, this, std::placeholders::_1));

    markerArrayPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);
    
    poseNavigator = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

    undockClient = rclcpp_action::create_client<irobot_create_msgs::action::Undock>(this, "/undock");
    dockClient = rclcpp_action::create_client<irobot_create_msgs::action::Dock>(this, "/dock");

    RCLCPP_INFO(get_logger(), "Turtlebot4 explorer waiting for nav2 stack..");
    poseNavigator->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "Waiting for undock action server..");

    undockClient->wait_for_action_server();
    auto undockGoal = irobot_create_msgs::action::Undock::Goal();
    auto undockGoalOptions = rclcpp_action::Client<irobot_create_msgs::action::Undock>::SendGoalOptions();
    // undockGoalOptions.result_callback = std::bind(&Explorer::start, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Sending undock command.");
    undockClient->async_send_goal(undockGoal, undockGoalOptions);

    timer = create_wall_timer(
    10s, [this]() {
        checkGoal();
    });
}

void Explorer::start(const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>::WrappedResult &result) {
    RCLCPP_INFO(get_logger(), "start: creating check goal timer.");
    timer = create_wall_timer(
    10s, [this]() {
        checkGoal();
    });
}

void Explorer::checkGoal() {
    RCLCPP_INFO(get_logger(), "checking goal.");

    if (!pose) {
        RCLCPP_INFO(get_logger(), "no pose received yet.");
        return;
    }

    findFrontiers();

    if (frontiers_.size() == 0) {
        RCLCPP_WARN(get_logger(), "No frontier found!");
        return;
    }

    if (pointInBB(currentGoalArea, frontiers_[0].centroid)) {
        RCLCPP_INFO(get_logger(), "Best frontier is already goal.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Best frontier is outside current goal area, canceling old and sending new goal.");
    auto future_cancel = poseNavigator->async_cancel_all_goals();

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = frontiers_[0].centroid;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    // send_goal_options.feedback_callback = std::bind(&Explorer::navigationFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.goal_response_callback = std::bind(&Explorer::navigationResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&Explorer::navigationResultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontiers_[0].centroid.x, frontiers_[0].centroid.y);
    poseNavigator->async_send_goal(goal, send_goal_options);

    currentGoalArea = frontierToBB(frontiers_[0]);

} 

void Explorer::explore() {

    // if (!pose || is_exploring) { return; }
    // is_exploring = true;

    // RCLCPP_INFO(get_logger(), "exploring..");

    // auto frontiers = findFrontiers();
    // if (frontiers.empty()) {
    //     RCLCPP_WARN(get_logger(), "No frontier found!");
    //     // stop();
    //     return;
    // }

    // RCLCPP_INFO(get_logger(), "frontiers:");
    // for (int i = 0; i < 3; i++) {
    //     auto newbb = frontierToBB(frontiers[i]);
    //     RCLCPP_INFO(get_logger(), "bb: x_min: %f, x_max: %f, y_min: %f, y_max: %f, size: %ld", newbb[0], newbb[1], newbb[2], newbb[3], frontiers[i].points.size());
    // }

    // drawMarkers(frontiers);

    // auto goal = nav2_msgs::action::NavigateToPose::Goal();
    // goal.pose.pose.position = frontiers[0].centroid;
    // goal.pose.pose.orientation.w = 1.;
    // goal.pose.header.frame_id = "map";

    // auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    // // send_goal_options.feedback_callback = std::bind(&Explorer::navigationFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    // send_goal_options.goal_response_callback = std::bind(&Explorer::navigationResponseCallback, this, std::placeholders::_1);
    // send_goal_options.result_callback = std::bind(&Explorer::navigationResultCallback, this, std::placeholders::_1);

    // RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontiers[0].centroid.x, frontiers[0].centroid.y);
    // poseNavigator->async_send_goal(goal, send_goal_options);

    // currentGoalArea = frontierToBB(frontiers[0]);
}

void Explorer::findFrontiers() {

    frontiers_.clear();
    
    const auto position = pose->pose.pose.position;
    unsigned int mx, my;

    if (!costmap.worldToMap(position.x, position.y, mx, my)) {
        RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
        return;
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap.getMutex()));

    auto map = costmap.getCharMap();

    std::vector<bool> frontier_flag(costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);
    std::vector<bool> visited_flag(costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);

    std::queue<unsigned int> bfs;

    unsigned int pos = costmap.getIndex(mx, my);
    unsigned int start;

    if (nearestCell(start, pos, nav2_costmap_2d::FREE_SPACE, costmap)) {
        bfs.push(start);
    } else {
        bfs.push(pos);
        RCLCPP_ERROR(get_logger(), "Could not find nearby clear cell to start search");
    }

    visited_flag[bfs.front()] = true;

    while (!bfs.empty()) {

        unsigned int idx = bfs.front();
        bfs.pop();

        for(unsigned nbr : nhood4(idx, costmap)) {
        cont:

            if (map[nbr] <= map[idx] && !visited_flag[nbr]) {
                visited_flag[nbr] = true;
                bfs.push(nbr);

            } else if (isAchievableFrontierCell(nbr, frontier_flag)) {
                frontier_flag[nbr] = true;

                Frontier frontier = buildNewFrontier(nbr, frontier_flag, position);

                for (const auto & bb : aborted) {
                    if(pointInBB(bb, frontier.centroid)) {
                        goto cont;
                    }
                }
                if (frontier.distance > min_dist &&
                    frontier.points.size() > min_size)
                {
                    frontiers_.push_back(frontier);
                }
            }
        }
    }

    std::sort (frontiers_.begin(), frontiers_.end(), compareFrontiers);

    drawMarkers(frontiers_);

}

Frontier Explorer::buildNewFrontier(unsigned int neighborCell, std::vector<bool> &frontier_flag, geometry_msgs::msg::Point robot_position) {
    
    Frontier output;

    frontier_flag[neighborCell] = true;

    unsigned int mx, my;
    double wx, wy;
    costmap.indexToCells(neighborCell, mx, my);
    costmap.mapToWorld(mx, my, wx, wy);

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

        for (unsigned int nbr: nhood8(idx, costmap)) {

            if (isAchievableFrontierCell(nbr, frontier_flag)) {

                frontier_flag[nbr] = true;

                costmap.indexToCells(nbr, mx, my);
                costmap.mapToWorld(mx, my, wx, wy);

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

    output.distance = std::sqrt(dx*dx + dy*dy);

    return output;
}

bool Explorer::isAchievableFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag) {
    
    auto map = costmap.getCharMap();

    if (map[idx] != nav2_costmap_2d::NO_INFORMATION || frontier_flag[idx]) {
        return false;
    }

    for(unsigned int nbr : nhood4(idx, costmap)) {
        if (map[nbr] == nav2_costmap_2d::FREE_SPACE) {
            return true;
        }
    }

    return false;
}

void Explorer::navigationResponseCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle) {
    if (goal_handle) {
        RCLCPP_INFO(get_logger(), "Goal accepted by server.");
    } else {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server.");
    }
}

void Explorer::navigationFeedbackCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> &feedback) {
    RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
}

void Explorer::navigationResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    // is_exploring = false;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal result: reached");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal result: aborted, marking as unreachable.");
            aborted.push_back(currentGoalArea);
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal result: canceled");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Goal result: Unknown");
            break;
    }
}

void Explorer::mapCallback(nav_msgs::msg::OccupancyGrid::UniquePtr occupancyGrid) {

    const auto occupancyGridInfo = occupancyGrid->info;
    costmap.resizeMap(occupancyGridInfo.width,
                        occupancyGridInfo.height,
                        occupancyGridInfo.resolution,
                        occupancyGridInfo.origin.position.x,
                        occupancyGridInfo.origin.position.y);

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());

    unsigned char *costmap_data = costmap.getCharMap();
    size_t costmap_size = costmap.getSizeInCellsX() * costmap.getSizeInCellsY();
    for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) {
        auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
        costmap_data[i] = costTranslationTable[cell_cost];
    }
}

void Explorer::poseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr poseMsg) {
    // RCLCPP_INFO(get_logger(), "poseCallback..");
    pose = move(poseMsg);
}

void Explorer::drawMarkers(const std::vector<Frontier> &frontiers) {

    clearMarkers();

    for (unsigned int i = 0; i < frontiers.size(); i++) {

        const Frontier& frontier = frontiers[i];

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

        std::vector<visualization_msgs::msg::Marker> &markers = markerArray.markers;
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
        markerArrayPublisher->publish(markerArray);
    }
}

void Explorer::clearMarkers() {

    for (auto &m: markerArray.markers) {
        m.action = visualization_msgs::msg::Marker::DELETE;
    }
    markerArrayPublisher->publish(markerArray);
    markerArray.markers.clear();
}


void Explorer::stop() {
    RCLCPP_INFO(get_logger(), "Explorer stopped..");
    
    poseSubscription.reset();
    mapSubscription.reset();
    poseNavigator->async_cancel_all_goals();
    saveMap();
    clearMarkers();
    timer->cancel();
}


void Explorer::saveMap() {
    auto mapSerializer = create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    auto serializePoseGraphRequest = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();

    serializePoseGraphRequest->filename = map_path;
    auto serializePoseResult = mapSerializer->async_send_request(serializePoseGraphRequest);

    auto map_saver = create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");
    auto saveMapRequest = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
    saveMapRequest->name.data = map_path;
    auto saveMapResult = map_saver->async_send_request(saveMapRequest);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto explorer = std::make_shared<Explorer>();
    rclcpp::spin(explorer);
    rclcpp::shutdown();
    return 0;
}