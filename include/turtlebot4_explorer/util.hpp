#ifndef TURTLEBOT4_EXPLORER__UTIL__HPP
#define TURTLEBOT4_EXPLORER__UTIL__HPP

#include <vector>
#include <array>
#include <queue>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"


struct Frontier {
    geometry_msgs::msg::Point centroid;
    std::vector<geometry_msgs::msg::Point> points;
    double distance;
};

bool compareFrontiers(Frontier& a, Frontier& b) {
    return a.points.size() < b.points.size();
}

bool isClose(geometry_msgs::msg::Point& a, geometry_msgs::msg::Point& b, double thresh = 0.05) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    if (std::sqrt(dx*dx + dy*dy) <= thresh) {
        return true;
    }
    return false;
}

std::array<double, 4> frontierToBB(Frontier& frontier) {
    double x_min = frontier.centroid.x, x_max = frontier.centroid.x;
    double y_min = frontier.centroid.y, y_max = frontier.centroid.y;
    for (const auto & p : frontier.points) {
        if (p.x < x_min) x_min = p.x;
        if (p.x > x_max) x_max = p.x;
        if (p.y < y_min) y_min = p.y;
        if (p.y > y_max) y_max = p.y;
    }
    return {x_min, x_max, y_min, y_max};
}

bool pointInBB(const std::array<double, 4>& bb, geometry_msgs::msg::Point& centroid) {
    double x_min = bb[0], x_max = bb[1], y_min = bb[2], y_max = bb[3];
    if (centroid.x > x_min && centroid.x < x_max &&
        centroid.y > y_min && centroid.y < y_max)
    {
            return true;
    } else {
        return false;
    }
}

bool frontierInBB(const std::array<double, 4>& bb, Frontier& frontier) {
    double x_min = bb[0], x_max = bb[1], y_min = bb[2], y_max = bb[3];
    for (const auto & p : frontier.points) {
        if (p.x < x_min || p.x > x_max || p.y < y_min || p.y > y_max) {
            return false;
        }
    }
    return true;
}

std::vector<unsigned int> nhood4(unsigned int idx, const nav2_costmap_2d::Costmap2D& costmap)
{
    std::vector<unsigned int> out;

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ -1)
    {
        return out;
    }

    if (idx % size_x_ > 0)
    {
        out.push_back(idx - 1);
    }
    if (idx % size_x_ < size_x_ - 1)
    {
        out.push_back(idx + 1);
    }
    if (idx >= size_x_)
    {
        out.push_back(idx - size_x_);
    }
    if (idx < size_x_*(size_y_-1))
    {
        out.push_back(idx + size_x_);
    }
    return out;
}

std::vector<unsigned int> nhood8(unsigned int idx, const nav2_costmap_2d::Costmap2D& costmap)
{
    std::vector<unsigned int> out = nhood4(idx, costmap);

    unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_ * size_y_ -1)
    {
        return out;
    }

    if (idx % size_x_ > 0 && idx >= size_x_)
    {
        out.push_back(idx - 1 - size_x_);
    }
    if (idx % size_x_ > 0 && idx < size_x_*(size_y_-1))
    {
        out.push_back(idx - 1 + size_x_);
    }
    if (idx % size_x_ < size_x_ - 1 && idx >= size_x_)
    {
        out.push_back(idx + 1 - size_x_);
    }
    if (idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1))
    {
        out.push_back(idx + 1 + size_x_);
    }

    return out;
}


bool nearestCell(unsigned int &result, unsigned int start, unsigned char val, const nav2_costmap_2d::Costmap2D& costmap) {

    const unsigned char* map = costmap.getCharMap();
    const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

    if (start >= size_x * size_y)
    {
        return false;
    }

    std::queue<unsigned int> bfs;
    std::vector<bool> visited_flag(size_x * size_y, false);

    bfs.push(start);
    visited_flag[start] = true;

    while (!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        if (map[idx] == val)
        {
            result = idx;
            return true;
        }

        for(unsigned nbr : nhood8(idx, costmap))
        {
            if (!visited_flag[nbr])
            {
                bfs.push(nbr);
                visited_flag[nbr] = true;
            }
        }
    }

    return false;
}

static std::array<unsigned char, 256> initTranslationTable() {
    std::array<unsigned char, 256> cost_translation_table{};

    // lineary mapped from [0..100] to [0..255]
    for (std::size_t i = 0; i < 256; ++i) {
        cost_translation_table[i] =
                static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
    }

    // special values:
    cost_translation_table[0] = nav2_costmap_2d::FREE_SPACE;
    cost_translation_table[99] = 253;
    cost_translation_table[100] = nav2_costmap_2d::LETHAL_OBSTACLE;
    cost_translation_table[static_cast<unsigned char>(-1)] = nav2_costmap_2d::NO_INFORMATION;

    return cost_translation_table;
}


#endif