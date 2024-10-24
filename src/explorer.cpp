#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include <cstddef>
#include <slam_toolbox/srv/detail/save_map__struct.hpp>

#include "turtlebot4_explorer/util.hpp"
#include "turtlebot4_explorer/explorer.hpp"

using namespace std::chrono_literals;


Explorer::Explorer()
: Node("Turtlebot4_explorer")
{
    RCLCPP_INFO(get_logger(), "Turtlebot4 explorer startup.");

    declare_parameter("map_path", rclcpp::ParameterValue(std::string("~")));
    declare_parameter("min_free", rclcpp::ParameterValue(2));
    declare_parameter("min_dist", rclcpp::ParameterValue(1.0));
    declare_parameter("min_weight", rclcpp::ParameterValue(0.5));

    get_parameter("min_weight", min_weight);
    get_parameter("min_dist", min_dist);
    get_parameter("min_free", min_free);
    get_parameter("map_path", map_path);

    RCLCPP_INFO(get_logger(), "min_weight: %f", min_weight);
    RCLCPP_INFO(get_logger(), "min_dist: %f", min_dist);
    RCLCPP_INFO(get_logger(), "min_free: %d", min_free);
    RCLCPP_INFO(get_logger(), "map_path: %s", map_path.c_str());

    poseSubscription = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", 10, std::bind(&Explorer::poseCallback, this, std::placeholders::_1));

    mapSubscription = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Explorer::mapCallback, this, std::placeholders::_1));

    markerArrayPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);
    
    poseNavigator = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this,
            "/navigate_to_pose");

    RCLCPP_INFO(get_logger(), "Turtlebot4 explorer waiting for nav2 stack..");
    poseNavigator->wait_for_action_server();
    RCLCPP_INFO(get_logger(), "Turtlebot4 explorer ready!");

    // timer = create_wall_timer(
    // 20s, [this]() {
    //     checkGoal();
    // });
}

void Explorer::explore() {

    if (isExploring || !pose) { return; }

    isExploring = true;

    auto frontiers = findFrontiers();
    if (frontiers.empty()) {
        isExploring = false;
        RCLCPP_WARN(get_logger(), "No frontier found!");
        stop();
        return;
    }

    Frontier * bestFrontier = &frontiers[0];
    double bestWeight = frontiers[0].weight;
    for (auto &frontier : frontiers) {
        if (frontier.weight > bestWeight) {
            bestFrontier = &frontier;
            bestWeight = frontier.weight;
        }
    }

    drawMarkers(frontiers);
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = bestFrontier->centroid;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = "map";

    RCLCPP_INFO(get_logger(), "Sending goal %f,%f", bestFrontier->centroid.x, bestFrontier->centroid.y);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    // send_goal_options.feedback_callback = std::bind(&Explorer::navigationFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.goal_response_callback = std::bind(&Explorer::navigationResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&Explorer::navigationResultCallback, this, std::placeholders::_1);

    poseNavigator->async_send_goal(goal, send_goal_options);

    currentGoal = bestFrontier->centroid;
}

void Explorer::navigationResponseCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle) {
    if (goal_handle) {
        RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    } else {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        isExploring = false;
    }
}

void Explorer::navigationFeedbackCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> &feedback) {
    RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
}

void Explorer::navigationResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    
    isExploring = false;
    saveMap();
    clearMarkers();
    explore();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal reached");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
    }
}

void Explorer::checkGoal() {

    if (isExploring) {

        RCLCPP_INFO(get_logger(), "checking goal..");

        unsigned int mx, my;

        if (costmap.worldToMap(currentGoal.x, currentGoal.y, mx, my)) {
            std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
            unsigned char *costmap_data = costmap.getCharMap();
            unsigned int pos = costmap.getIndex(mx, my);
            if (costmap_data[pos] != nav2_costmap_2d::NO_INFORMATION) {
                RCLCPP_INFO(get_logger(), "Goal cell got explored, looking for new goals..");
                poseNavigator->async_cancel_all_goals();
                explore();
            } else {
                RCLCPP_INFO(get_logger(), "goal still unkown");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "[checkGoal] costmap.worldToMap() failed, robot out of costmap bounds, cannot search for frontiers");
        }
    } else {
        explore();
    }

}

void Explorer::mapCallback(nav_msgs::msg::OccupancyGrid::UniquePtr occupancyGrid) {

    // RCLCPP_INFO(get_logger(), "explorer map callback: pose is null: %d", (pose == nullptr));

    if (pose == nullptr) { return; }

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

std::vector<Frontier> Explorer::findFrontiers() {
    std::vector<Frontier> frontier_list;
    const auto position = pose->pose.pose.position;
    unsigned int mx, my;

    if (!costmap.worldToMap(position.x, position.y, mx, my)) {
        RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap.getMutex()));

    auto map = costmap.getCharMap();

    std::vector<bool> frontier_flag(costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);
    std::vector<bool> visited_flag(costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);

    std::queue<unsigned int> bfs;

    unsigned int pos = costmap.getIndex(mx, my);
    bfs.push(pos);
    visited_flag[bfs.front()] = true;

    while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

        for (unsigned nbr: nhood8(idx, costmap)) {

            if (map[nbr] == nav2_costmap_2d::FREE_SPACE && !visited_flag[nbr]) {
                visited_flag[nbr] = true;
                bfs.push(nbr);

            } else if (isAchievableFrontierCell(nbr, frontier_flag)) {
                frontier_flag[nbr] = true;
                const Frontier frontier = buildNewFrontier(nbr, frontier_flag);

                double distance = sqrt(pow((double(frontier.centroid.x) - double(position.x)), 2.0) +
                                        pow((double(frontier.centroid.y) - double(position.y)), 2.0));

                if (distance < min_dist) { continue; }

                if (frontier.weight >= min_weight) {
                    frontier_list.push_back(frontier);
                }
            }
        }
    }

    return frontier_list;
}

bool Explorer::isAchievableFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag) {

    auto map = costmap.getCharMap();

    if (map[idx] != nav2_costmap_2d::NO_INFORMATION || frontier_flag[idx]) {
        return false;
    }

    int freeCount = 0;
    for (unsigned int nbr: nhood8(idx, costmap)) {
        if (map[nbr] == nav2_costmap_2d::FREE_SPACE) {
                if (++freeCount >= min_free) {
                return true;
            }
        }
    }

    return false;
}

Frontier Explorer::buildNewFrontier(unsigned int neighborCell, std::vector<bool> &frontier_flag) {
    
    Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;

    std::queue<unsigned int> bfs;
    bfs.push(neighborCell);

    while (!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

        // try adding cells in 8-connected neighborhood to frontier
        for (unsigned int nbr: nhood8(idx, costmap)) {
            // check if neighbour is a potential frontier cell
            if (isAchievableFrontierCell(nbr, frontier_flag)) {
                // mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx, my;
                double wx, wy;
                costmap.indexToCells(nbr, mx, my);
                costmap.mapToWorld(mx, my, wx, wy);

                geometry_msgs::msg::Point point;
                point.x = wx;
                point.y = wy;
                output.points.push_back(point);

                // update centroid of frontier
                output.centroid.x += wx;
                output.centroid.y += wy;

                bfs.push(nbr);
            }
        }
    }

    // average out frontier centroid
    output.centroid.x /= output.points.size();
    output.centroid.y /= output.points.size();

    output.weight = output.points.size() * costmap.getResolution();

    return output;
}

void Explorer::drawMarkers(const std::vector<Frontier> &frontiers) {
    for (const auto &frontier: frontiers) {
        // RCLCPP_INFO(get_logger(), "visualising %f,%f ", frontier.centroid.x, frontier.centroid.y);
        std_msgs::msg::ColorRGBA green;
        green.r = 0;
        green.g = 1.0;
        green.b = 0;
        green.a = 1.0;

        std::vector<visualization_msgs::msg::Marker> &markers = markerArray.markers;
        visualization_msgs::msg::Marker m;

        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.frame_locked = true;

        m.action = visualization_msgs::msg::Marker::ADD;
        m.ns = "frontiers";
        m.id = ++markerId;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position = frontier.centroid;
        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;
        m.color = green;
        markers.push_back(m);
        markerArrayPublisher->publish(markerArray);
    }
}

void Explorer::clearMarkers() {

    visualization_msgs::msg::Marker m;

    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.frame_locked = true;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    m.ns = "frontiers";
    m.id = ++markerId;

    markerArray.markers.push_back(m);
    markerArrayPublisher->publish(markerArray);

    // for (auto &m: markerArray_.markers) {
    //     m.action = Marker::DELETE;
    // }
    // markerArrayPublisher_->publish(markerArray_);
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