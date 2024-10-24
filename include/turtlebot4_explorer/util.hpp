#ifndef TURTLEBOT4_EXPLORER__UTIL__HPP
#define TURTLEBOT4_EXPLORER__UTIL__HPP

#include <vector>
#include <array>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point.hpp"

struct Frontier {
    geometry_msgs::msg::Point centroid;
    std::vector<geometry_msgs::msg::Point> points;
    double weight;
};


std::vector<unsigned int> nhood8(unsigned int idx, const nav2_costmap_2d::Costmap2D &costmap) {
    
    std::vector<unsigned int> out;
    
    unsigned int mx, my;
    costmap.indexToCells(idx, mx, my);
    
    const std::pair<int, int> directions[] = {
            std::pair(-1, -1),
            std::pair(-1, 1),
            std::pair(1, -1),
            std::pair(1, 1),
            std::pair(1, 0),
            std::pair(-1, 0),
            std::pair(0, 1),
            std::pair(0, -1)
    };

    for (const auto &d: directions) {
        int newX = mx + d.first;
        int newY = my + d.second;
        if (newX > -1 && newX < costmap.getSizeInCellsX() &&
            newY > -1 && newY < costmap.getSizeInCellsY()) {
            out.push_back(costmap.getIndex(newX, newY));
        }
    }
    return out;
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