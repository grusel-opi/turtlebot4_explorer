#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include <cstddef>
#include <slam_toolbox/srv/detail/save_map__struct.hpp>
#include <vector>

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
    declare_parameter("pose_topic",rclcpp::ParameterValue(std::string("/pose")));
    declare_parameter("lower_cost_bound", rclcpp::ParameterValue(40));
    declare_parameter("upper_cost_bound", rclcpp::ParameterValue(150));

    get_parameter("min_size", min_size_);
    get_parameter("min_dist", min_dist_);
    get_parameter("map_path", map_path_);
    get_parameter("pose_topic", pose_topic_);

    RCLCPP_INFO(get_logger(), "min_size: %d", min_size_);
    RCLCPP_INFO(get_logger(), "min_dist: %f", min_dist_);
    RCLCPP_INFO(get_logger(), "map_path: %s", map_path_.c_str());
    RCLCPP_INFO(get_logger(), "pose_topic: %s", pose_topic_.c_str());

    pose_navigator_ = rclcpp_action::create_client<NavAction>(this, "/navigate_to_pose");
    
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_, 10, std::bind(&Explorer::poseCallback, this, std::placeholders::_1));

    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Explorer::mapCallback, this, std::placeholders::_1));

    marker_array_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);
    
    cost_translation_table_ = initTranslationTable();

    start_pose_ = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

    map_received_ = false;

    is_navigating_ = false;

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

    RCLCPP_INFO(get_logger(), "Remembering start pose.");
    start_pose_->pose.pose.position = pose_->pose.pose.position;
    start_pose_->pose.pose.orientation = pose_->pose.pose.orientation;

    RCLCPP_INFO(get_logger(), "Starting exploration!");
    
    // explore();
    stop();
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
        if (map[nbr] < 200) { // TODO: make this thresh configurable?
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
            RCLCPP_INFO(get_logger(), "[RESULT] Goal result: reached. Still unexplored: %d", checkGoal());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            {
                auto bb = frontierToBB(current_goal_, costmap_.getResolution());
                RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: aborted, marking as unreachable: x:%f-%f, y: %f-%f Still unexplored: %d", bb[0], bb[1], bb[2], bb[3], checkGoal());
                aborted_.push_back(bb);
            }
            break;
        case rclcpp_action::ResultCode::CANCELED:
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

    RCLCPP_INFO(get_logger(), "Sending return to home goal %f,%f", goal.pose.pose.position.x, goal.pose.pose.position.y);
    auto return_goal_handle = pose_navigator_->async_send_goal(goal, send_goal_options);
}


void Explorer::saveMap() {
    auto mapSerializer = create_client<slam_toolbox::srv::SerializePoseGraph>("/slam_toolbox/serialize_map");
    auto serializePoseGraphRequest = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();

    serializePoseGraphRequest->filename = map_path_;
    RCLCPP_INFO(get_logger(), "Sending request to /slam_toolbox/serialize_map");
    auto serializePoseResult = mapSerializer->async_send_request(serializePoseGraphRequest);

    auto map_saver = create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");
    auto saveMapRequest = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
    saveMapRequest->name.data = map_path_;
    RCLCPP_INFO(get_logger(), "Sending request to /slam_toolbox/save_map");
    auto saveMapResult = map_saver->async_send_request(saveMapRequest);
}


void Explorer::calculateCoverage() {

    RCLCPP_INFO(get_logger(), "[calculateCoverage]");

    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr client = create_client<nav2_msgs::srv::GetCostmap>("/global_costmap/get_costmap");
    auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    
    RCLCPP_INFO(get_logger(), "waiting for /global_costmap/get_costmap sercive now..");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(get_logger(), "service not available, waiting again 1s...");
    }

    RCLCPP_INFO(get_logger(), "sending request to /global_costmap/get_costmap");

    auto async_cb = [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture result) {
        RCLCPP_INFO(get_logger(), "SUCCESS?");

        nav2_costmap_2d::Costmap2D costmap;
        nav2_msgs::msg::Costmap map;

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
            costmap_data[i] = map.data[i];
        }

        std::vector<geometry_msgs::msg::Point> positions;
        coverage_positions_sorted_.clear();

        if (!randomWalkSampling(positions, costmap)) {
            RCLCPP_ERROR(get_logger(), "Error calculating coverage poses!");
            return;
        }
        
        cheapTSP(positions);

        for (unsigned int i = 0; i < coverage_positions_sorted_.size(); i++) {

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
            m.pose.position = coverage_positions_sorted_[i];
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            m.color = color;
            markers.push_back(m);
        }
        marker_array_publisher_->publish(marker_array);

        RCLCPP_INFO(get_logger(), "published positions number: %ld", marker_array.markers.size());

        current_coverage_pose_nr_ = 0;
        executeCoverage();
    };

    client->async_send_request(request, async_cb);
}

bool Explorer::randomWalkSampling(std::vector<geometry_msgs::msg::Point>& positions, nav2_costmap_2d::Costmap2D& costmap) {

    const auto position = start_pose_->pose.pose.position;
    unsigned int mx, my;

    if (!costmap.worldToMap(position.x, position.y, mx, my)) {
        RCLCPP_ERROR(get_logger(), "Robot start position out of costmap bounds, should not be possible..");
        return false;
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap.getMutex()));

    auto map = costmap.getCharMap();

    std::vector<bool> frontier_flag(costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);
    std::vector<bool> visited_flag(costmap.getSizeInCellsX() * costmap.getSizeInCellsY(), false);
    
    std::queue<unsigned int> bfs;

    unsigned char cost = costmap.getCost(mx, my);
    unsigned int pos_idx = costmap.getIndex(mx, my);

    unsigned char upper_cost_bound = 150;
    unsigned char lower_cost_bound = 80;

    RCLCPP_INFO(get_logger(), "start cost: %u", cost);

    while (cost > upper_cost_bound || cost < lower_cost_bound) {
        for (unsigned nbr : nhood4(pos_idx, costmap)) {
            if ((cost > upper_cost_bound && map[nbr] <= cost) || (cost < lower_cost_bound && map[nbr] >= cost)) {
                cost = map[nbr];
                pos_idx = nbr;
            }
        }
    }

    RCLCPP_INFO(get_logger(), "new cost: %u", cost);

    bfs.push(pos_idx);

    geometry_msgs::msg::Point pos;
    costmap.indexToCells(pos_idx, mx, my);
    costmap.mapToWorld(mx, my, pos.x, pos.y);

    visited_flag[bfs.front()] = true;

    // int sample_dist = 10;
    // int counter = 0;

    while (!bfs.empty()) {

        unsigned int idx = bfs.front();
        bfs.pop();

        // counter++;
        if (map[idx] <= upper_cost_bound && map[idx] >= lower_cost_bound /*  && counter > sample_dist */) {
            costmap.indexToCells(idx, mx, my);
            costmap.mapToWorld(mx, my, pos.x, pos.y);
            positions.push_back(pos);
            // counter = 0;
        }

        for (unsigned nbr : nhood4(idx, costmap)) {

            if (!visited_flag[nbr] && map[nbr] <= upper_cost_bound /* && map[nbr] >= lower_cost_bound*/) {
                visited_flag[nbr] = true;
                bfs.push(nbr);
            }
        }
    }
    return true;
}

void Explorer::executeCoverage() {

    geometry_msgs::msg::Point next_goal = coverage_positions_sorted_[current_coverage_pose_nr_];

    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.pose.position = next_goal;
    goal.pose.pose.orientation.w = 1.;
    goal.pose.header.frame_id = "map";

    auto send_goal_options = NavClient::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const auto& goal_handle) {
        if (goal_handle) {
            RCLCPP_INFO(get_logger(), "[RESPONSE] Goal accepted by server.");
        } else {
            RCLCPP_ERROR(get_logger(), "[RESPONSE] Goal was rejected by server.");
            current_coverage_pose_nr_++;
            executeCoverage();
        }
    };

    send_goal_options.result_callback = [this](const auto& result) {
        if (result.goal_id != future_goal_handle_.get()->get_goal_id()) {
            RCLCPP_DEBUG(get_logger(),
            "[RESULT] Goal IDs do not match for the current goal handle and received result."
            "Ignoring likely due to receiving result for an old goal.");
            return;
        }

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "[RESULT] Goal result: reached.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: aborted, continuing with next..");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: canceled.");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "[RESULT] Goal result: Unknown");
                break;
        }
        current_coverage_pose_nr_++;
        executeCoverage();
    };

    RCLCPP_INFO(get_logger(), "[REQUEST] Sending goal %f,%f", next_goal.x, next_goal.y);
    future_goal_handle_ = pose_navigator_->async_send_goal(goal, send_goal_options);
}


void Explorer::cheapTSP(std::vector<geometry_msgs::msg::Point>& positions) {
    
    geometry_msgs::msg::Point current_pos = pose_->pose.pose.position;

    coverage_positions_sorted_.push_back(current_pos);

    int best_next_idx = 0;
    double best_dist = 10e10f;

    int todo = positions.size();
    std::vector<bool> planned(todo, false);
    int done = 0;

    while (done < todo) {

        for (int i = 0; i < todo; i++) {
            if (!planned[i]) {
                double tmp_dist = std::sqrt(std::pow(current_pos.x - positions[i].x, 2) + std::pow(current_pos.y - positions[i].y, 2));
                if (tmp_dist < best_dist) {
                    best_dist = tmp_dist;
                    best_next_idx = i;
                }
            }
        }

        coverage_positions_sorted_.push_back(positions[best_next_idx]);
        current_pos = positions[best_next_idx];
        RCLCPP_INFO(get_logger(), "Next goal: %f, %f; dist: %f", current_pos.x, current_pos.y, best_dist);
        
        done++;
        planned[best_next_idx] = true;
        best_dist = 10e10f;
    }

    coverage_positions_sorted_.push_back(pose_->pose.pose.position);

}

int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
    auto explorer = std::make_shared<Explorer>();
    
    explorer->start();
    
    rclcpp::spin(explorer);
    rclcpp::shutdown();
    return 0;
}