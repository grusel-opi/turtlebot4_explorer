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
    declare_parameter("min_dist", rclcpp::ParameterValue(1.0));
    declare_parameter("min_size", rclcpp::ParameterValue(5));

    get_parameter("min_size", min_size_);
    get_parameter("min_dist", min_dist_);
    get_parameter("map_path", map_path_);

    RCLCPP_INFO(get_logger(), "min_size: %d", min_size_);
    RCLCPP_INFO(get_logger(), "min_dist: %f", min_dist_);
    RCLCPP_INFO(get_logger(), "map_path: %s", map_path_.c_str());

    pose_navigator_ = rclcpp_action::create_client<NavAction>(this, "/navigate_to_pose");
    
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", 10, std::bind(&Explorer::poseCallback, this, std::placeholders::_1));

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Explorer::mapCallback, this, std::placeholders::_1));

    marker_array_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);
    
    undock_client_ = rclcpp_action::create_client<UndockAction>(this, "/undock");
    dock_client_ = rclcpp_action::create_client<DockAction>(this, "/dock");

    cost_translation_table_ = initTranslationTable();

    start_pose_ = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

    current_goal_status_.status = ActionStatus::IDLE;

    map_received_ = false;
}

void Explorer::start() {
    
    RCLCPP_INFO(get_logger(), "Waiting for nav2 stack..");
    pose_navigator_->wait_for_action_server();
   
    rclcpp::WallRate wait_for_pose_rate(1);
    while (rclcpp::ok() && !pose_) {
        RCLCPP_INFO(get_logger(), "Waiting to receive pose message..");
        rclcpp::spin_some(shared_from_this());
        wait_for_pose_rate.sleep();
    }

    // lets not do automatic docking and undocking since these maneuvers are very rough (distorting the map) and not modifiable (thx irobot people)

    // RCLCPP_INFO(get_logger(), "Waiting for undock action server..");
    // undock_client_->wait_for_action_server();
    // RCLCPP_INFO(get_logger(), "Sending undock command.");
    // auto undock_goal = UndockAction::Goal();
    // auto undock_goal_options = UndockClient::SendGoalOptions();
    // undock_goal_options.result_callback = [this](const auto msg){ RCLCPP_INFO(get_logger(), "Undocking result: is_docked: %d", msg.result->is_docked);};
    // auto undock_future = undock_client_->async_send_goal(undock_goal, undock_goal_options);
    // rclcpp::spin_until_future_complete(shared_from_this(), undock_future);
    // rclcpp::spin_some(shared_from_this());

    RCLCPP_INFO(get_logger(), "Remembering start pose.");
    start_pose_->pose.pose.position = pose_->pose.pose.position;
    start_pose_->pose.pose.orientation = pose_->pose.pose.orientation;

    RCLCPP_INFO(get_logger(), "Starting exploration!");
    explore();
}

void Explorer::explore() {

    findFrontiers();

    if (frontiers_.size() == 0) {
        RCLCPP_WARN(get_logger(), "No frontier found!");
        stop();
        return;
    }

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = frontiers_[0].centroid;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = "map";

    auto send_goal_options = NavClient::SendGoalOptions();
    // send_goal_options.feedback_callback = std::bind(&Explorer::navigationFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.goal_response_callback = std::bind(&Explorer::navigationResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&Explorer::navigationResultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontiers_[0].centroid.x, frontiers_[0].centroid.y);
    future_goal_handle_ = pose_navigator_->async_send_goal(goal, send_goal_options);
    current_goal_status_.status = ActionStatus::PROCESSING;
    current_goal_ = frontiers_[0];
} 

bool Explorer::checkGoal() {

    unsigned int mx, my, free_count = 0;
    unsigned char *costmap_data = costmap_.getCharMap();
    const int thresh = 5;

    for (const auto & p : current_goal_.points) {
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

void Explorer::findFrontiers() {

    frontiers_.clear();
    
    const auto position = pose_->pose.pose.position;
    unsigned int mx, my;

    if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
        RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
        return;
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

    auto map = costmap_.getCharMap();

    std::vector<bool> frontier_flag(costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY(), false);
    std::vector<bool> visited_flag(costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY(), false);

    std::queue<unsigned int> bfs;

    unsigned int pos = costmap_.getIndex(mx, my);
    unsigned int start;

    if (nearestCell(start, pos, nav2_costmap_2d::FREE_SPACE, costmap_)) {
        bfs.push(start);
    } else {
        bfs.push(pos);
        RCLCPP_ERROR(get_logger(), "Could not find nearby clear cell to start search");
    }

    visited_flag[bfs.front()] = true;

    while (!bfs.empty()) {

        unsigned int idx = bfs.front();
        bfs.pop();

        for(unsigned nbr : nhood4(idx, costmap_)) {

            if (map[nbr] <= map[idx] && !visited_flag[nbr]) {
                visited_flag[nbr] = true;
                bfs.push(nbr);

            } else if (isAchievableFrontierCell(nbr, frontier_flag)) {
                frontier_flag[nbr] = true;

                Frontier frontier = buildNewFrontier(nbr, frontier_flag, position);
                bool aborted = false;
                for (const auto & bb : aborted_) {
                    if(pointInBB(bb, frontier.centroid)) {
                        RCLCPP_ERROR(get_logger(), "abort check: (%f, %f) is in area %f - %f, %f - %f", frontier.centroid.x, frontier.centroid.y, bb[0], bb[1], bb[2], bb[3]);
                        aborted = true;
                        break;
                    }
                }
                if (!aborted &&
                    frontier.distance > min_dist_ &&
                    frontier.points.size() > min_size_)
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

        for (unsigned int nbr: nhood8(idx, costmap_)) {

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

    output.distance = std::sqrt(dx*dx + dy*dy);

    return output;
}

bool Explorer::isAchievableFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag) {
    
    auto map = costmap_.getCharMap();

    if (map[idx] != nav2_costmap_2d::NO_INFORMATION || frontier_flag[idx]) {
        return false;
    }

    for(unsigned int nbr : nhood4(idx, costmap_)) {
        if (map[nbr] == nav2_costmap_2d::FREE_SPACE) {
            return true;
        }
    }

    return false;
}

void Explorer::navigationResponseCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle) {
    if (goal_handle) {
        RCLCPP_INFO(get_logger(), "[RESPONSE] Goal accepted by server.");
    } else {
        RCLCPP_ERROR(get_logger(), "[RESPONSE] Goal was rejected by server.");
        current_goal_status_.status = ActionStatus::FAILED;
        explore();
    }
}

void Explorer::navigationFeedbackCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> &feedback) {
    RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
}

void Explorer::navigationResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    if (result.goal_id != future_goal_handle_.get()->get_goal_id()) {
        RCLCPP_DEBUG(get_logger(),
        "[RESULT] Goal IDs do not match for the current goal handle and received result."
        "Ignoring likely due to receiving result for an old goal.");
      return;
    }

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            current_goal_status_.status = ActionStatus::SUCCEEDED;
            RCLCPP_INFO(get_logger(), "[RESULT] Goal result: reached. Still unexplored: %d", checkGoal());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            current_goal_status_.status = ActionStatus::FAILED;
            // current_goal_status_.error_code = result.result.error_code;
            {
                auto bb = frontierToBB(current_goal_, costmap_.getResolution());
                RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: aborted, marking as unreachable: x:%f-%f, y: %f-%f Still unexplored: %d", bb[0], bb[1], bb[2], bb[3], checkGoal());
                aborted_.push_back(bb);
            }
            break;
        case rclcpp_action::ResultCode::CANCELED:
            current_goal_status_.status = ActionStatus::FAILED;
            RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: canceled. Still unexplored: %d", checkGoal());
            break;
        default:
            RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: Unknown");
            break;
    }
    explore();
}

void Explorer::mapCallback(nav_msgs::msg::OccupancyGrid::UniquePtr occupancyGrid) {

    const auto occupancyGridInfo = occupancyGrid->info;
    costmap_.resizeMap(occupancyGridInfo.width,
                        occupancyGridInfo.height,
                        occupancyGridInfo.resolution,
                        occupancyGridInfo.origin.position.x,
                        occupancyGridInfo.origin.position.y);

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_.getMutex());

    unsigned char *costmap_data = costmap_.getCharMap();
    size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
    for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) {
        auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
        costmap_data[i] = cost_translation_table_[cell_cost];
    }

    map_received_ = true;

}

void Explorer::calculateGridPattern() {

    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr client = create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    nav2_msgs::msg::CostmapMetaData meta_data;
    request.get()->specs = meta_data;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }

    nav2_costmap_2d::Costmap2D costmap;
    nav2_msgs::msg::Costmap map;
    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(get_logger(), "SUCCESS");

        map = result.get()->map;

        const auto meta_data = map.metadata;
        costmap.resizeMap(meta_data.size_x,
                          meta_data.size_y,
                          meta_data.resolution,
                          meta_data.origin.position.x,
                          meta_data.origin.position.y);


        unsigned char *costmap_data = costmap.getCharMap();
        size_t costmap_size = costmap.getSizeInCellsX() * costmap.getSizeInCellsY();
        for (size_t i = 0; i < costmap_size && i < map.data.size(); ++i) {
            auto cell_cost = static_cast<unsigned char>(map.data[i]);
            costmap_data[i] = cost_translation_table_[cell_cost];
        }

    } else {
        RCLCPP_ERROR(get_logger(), "Failed to call service");
        return;
    }

    const double step_size_w = 0.75;
    const unsigned int step_size_m = (unsigned int) step_size_w / costmap.getResolution();
    const double size_x_m = costmap.getSizeInCellsX();
    const double size_y_m = costmap.getSizeInCellsY();

    std::vector<geometry_msgs::msg::Point> positions;

    unsigned int mx, my;

    RCLCPP_INFO(get_logger(), "starting search");

    for (mx = 0; mx < size_x_m; mx += step_size_m) {
        for (my = 0; my < size_y_m; my += step_size_m) {

            unsigned char cost = costmap.getCost(mx, my);
            unsigned char best_cost = cost;
            unsigned int best_mx = mx, best_my = my;
            bool found_next = false;
            int count = 0;

            RCLCPP_INFO(get_logger(), "cost for %d, %d: %d", mx, my, cost);

            if (cost < 100) {

                while (best_cost > 20 && count < 20) {
                    
                    for (int dx = -1; dx < 2; dx++) {
                        for (int dy = -1; dy < 2; dy++) {

                            count++;

                            unsigned int nbr_x = best_mx + dx, nbr_y = best_my + dy;

                            if (nbr_x >= size_x_m || nbr_y >= size_y_m) {
                                continue;
                            }

                            unsigned char nbr_cost = costmap.getCost(nbr_x, nbr_y);

                            RCLCPP_INFO(get_logger(), "searching: cost for %d, %d: %d", nbr_x, nbr_y, nbr_cost);

                            if (nbr_cost < best_cost) {
                                best_my = nbr_y;
                                best_mx = nbr_x;
                                best_cost = nbr_cost;
                                found_next = true;
                            }
                        }
                    }
                    if (!found_next) {
                        RCLCPP_INFO(get_logger(), "could not find a cost gradient!");
                        break;
                    }
                }
                if (found_next) {
                    geometry_msgs::msg::Point p;
                    costmap.mapToWorld(best_mx, best_my, p.x, p.y);
                    positions.push_back(p);
                }
            }
        }
    }

    for (unsigned int i = 0; i < positions.size(); i++) {

        std_msgs::msg::ColorRGBA color;

        color.r = 1.0;
        color.g = 0;
        color.b = 0;
        color.a = 1.0;

        std::vector<visualization_msgs::msg::Marker> &markers = marker_array.markers;
        visualization_msgs::msg::Marker m;

        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.frame_locked = true;

        m.action = visualization_msgs::msg::Marker::ADD;
        m.ns = "grid_pattern";
        m.id = i;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.pose.position = positions[i];
        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;
        m.color = color;
        markers.push_back(m);
    }
    marker_array_publisher_->publish(marker_array);

    RCLCPP_INFO(get_logger(), "published positions number: %ld", marker_array.markers.size());

}

void Explorer::poseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr poseMsg) {
    // RCLCPP_INFO(get_logger(), "poseCallback..");
    pose_ = move(poseMsg);
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

        std::vector<visualization_msgs::msg::Marker> &markers = marker_array.markers;
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
        marker_array_publisher_->publish(marker_array);
    }
}

void Explorer::clearMarkers() {

    for (auto &m: marker_array.markers) {
        m.action = visualization_msgs::msg::Marker::DELETE;
    }
    marker_array_publisher_->publish(marker_array);
    marker_array.markers.clear();
}


void Explorer::stop() {
    RCLCPP_INFO(get_logger(), "Explorer stopped, returning to start pose.");

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = start_pose_->pose.pose.position;
    goal.pose.pose.orientation = start_pose_->pose.pose.orientation;
    goal.behavior_tree = "/home/mayerfel/ws/autonomous_acquisition/bt/nav2home.xml";
    goal.pose.header.frame_id = "map";

    auto send_goal_options = NavClient::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const auto msg) {
        if (msg) {
            RCLCPP_INFO(get_logger(), "Return to start goal accepted by server.");
        } else {
            RCLCPP_ERROR(get_logger(), "Return to start goal was rejected by server.");
        }
    };
   
    send_goal_options.result_callback = [this](const auto result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Return to start goal result: reached.");

            // {
            //     RCLCPP_INFO(get_logger(), "Waiting for dock action server..");
            //     dock_client_->wait_for_action_server();

            //     RCLCPP_INFO(get_logger(), "Sending dock command.");
            //     auto dock_goal = DockAction::Goal();
            //     auto dock_goal_options = DockClient::SendGoalOptions();
            //     dock_goal_options.result_callback = [this](const auto msg) {
            //         RCLCPP_INFO(get_logger(), "Docking result: is_docked: %d", msg.result->is_docked);
            //         RCLCPP_INFO(get_logger(), "Should be docked now, stopping and saving map.");

            //         pose_subscription_.reset();
            //         map_subscription_.reset();
            //         pose_navigator_->async_cancel_all_goals();
            //         saveMap();
            //         clearMarkers();

            //         RCLCPP_INFO(get_logger(), "All done, send SIGINT now pls..");

            //     };
            //     auto dock_future = dock_client_->async_send_goal(dock_goal, dock_goal_options);
            // }

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
        RCLCPP_INFO(get_logger(), "All done, send SIGINT now pls..");
    };

    RCLCPP_INFO(get_logger(), "Sending return to home goal %f,%f", goal.pose.pose.position.x, goal.pose.pose.position.y);
    auto return_goal_handle = pose_navigator_->async_send_goal(goal, send_goal_options);
}


void Explorer::saveMap() {
    auto mapSerializer = create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    auto serializePoseGraphRequest = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();

    serializePoseGraphRequest->filename = map_path_;
    auto serializePoseResult = mapSerializer->async_send_request(serializePoseGraphRequest);

    auto map_saver = create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");
    auto saveMapRequest = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
    saveMapRequest->name.data = map_path_;
    auto saveMapResult = map_saver->async_send_request(saveMapRequest);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto explorer = std::make_shared<Explorer>();
    
    // explorer->start();
    explorer->calculateGridPattern();
    
    rclcpp::spin(explorer);
    rclcpp::shutdown();
    return 0;
}