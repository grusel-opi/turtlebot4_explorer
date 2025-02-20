#ifndef TURTLEBOT4_EXPLORER__UTIL__HPP
#define TURTLEBOT4_EXPLORER__UTIL__HPP

#include <array>
#include <cmath>
#include <math.h>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


struct Frontier {
  geometry_msgs::msg::Point centroid;
  std::vector<geometry_msgs::msg::Point> points;
  double distance;
};

bool compareFrontiers(Frontier &a, Frontier &b) {
  return a.distance < b.distance;
}

std::array<double, 4> frontierToBB(Frontier &frontier, float resolution) {
  double x_min = frontier.centroid.x, x_max = frontier.centroid.x;
  double y_min = frontier.centroid.y, y_max = frontier.centroid.y;
  for (const auto &p : frontier.points) {
    if (p.x < x_min)
      x_min = p.x;
    if (p.x > x_max)
      x_max = p.x;
    if (p.y < y_min)
      y_min = p.y;
    if (p.y > y_max)
      y_max = p.y;
  }
  return {x_min - resolution / 2., x_max + resolution / 2.,
          y_min - resolution / 2., y_max + resolution / 2.};
}

bool pointInBB(const std::array<double, 4> &bb,
               geometry_msgs::msg::Point &centroid) {
  double x_min = bb[0], x_max = bb[1], y_min = bb[2], y_max = bb[3];
  if (centroid.x > x_min && centroid.x < x_max && centroid.y > y_min &&
      centroid.y < y_max) {
    return true;
  } else {
    return false;
  }
}

std::vector<unsigned int> nhood4(unsigned int idx,
                                 const nav2_costmap_2d::Costmap2D &costmap) {
  std::vector<unsigned int> out;

  unsigned int size_x_ = costmap.getSizeInCellsX(),
               size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }

  if (idx % size_x_ > 0) {
    out.push_back(idx - 1);
  }
  if (idx % size_x_ < size_x_ - 1) {
    out.push_back(idx + 1);
  }
  if (idx >= size_x_) {
    out.push_back(idx - size_x_);
  }
  if (idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + size_x_);
  }
  return out;
}

std::vector<unsigned int> nhood8(unsigned int idx,
                                 const nav2_costmap_2d::Costmap2D &costmap) {
  std::vector<unsigned int> out = nhood4(idx, costmap);

  unsigned int size_x_ = costmap.getSizeInCellsX(),
               size_y_ = costmap.getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }

  if (idx % size_x_ > 0 && idx >= size_x_) {
    out.push_back(idx - 1 - size_x_);
  }
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx - 1 + size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    out.push_back(idx + 1 - size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + 1 + size_x_);
  }

  return out;
}

bool isCostBorderCell(unsigned int idx, unsigned int &nbr,
                      const nav2_costmap_2d::Costmap2D &costmap) {

  if (costmap.getCost(idx) == 0) {
    for (unsigned nbr_idx : nhood8(idx, costmap)) {
        if (costmap.getCost(nbr_idx) > 0) {
            nbr = nbr_idx;
            return true;
        }
    }
  }

  return false;
}

void costmapMsgTo2D(nav2_msgs::msg::Costmap &map_msg,
                    nav2_costmap_2d::Costmap2D &costmap) {

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap.getMutex()));

  const auto meta_data = map_msg.metadata;
  costmap.resizeMap(meta_data.size_x, meta_data.size_y, meta_data.resolution,
                    meta_data.origin.position.x, meta_data.origin.position.y);

  unsigned char *costmap_data = costmap.getCharMap();
  size_t costmap_size = costmap.getSizeInCellsX() * costmap.getSizeInCellsY();
  for (size_t i = 0; i < costmap_size && i < map_msg.data.size(); ++i) {
    costmap_data[i] = map_msg.data[i];
  }
}

bool nearestCell(unsigned int &result, unsigned int start,
                 unsigned char lower_val, unsigned char upper_val,
                 const nav2_costmap_2d::Costmap2D &costmap) {

  const unsigned char *map = costmap.getCharMap();
  const unsigned int size_x = costmap.getSizeInCellsX(),
                     size_y = costmap.getSizeInCellsY();

  if (start >= size_x * size_y) {
    return false;
  }

  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x * size_y, false);

  bfs.push(start);
  visited_flag[start] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    if (map[idx] <= upper_val && map[idx] >= lower_val) {
      result = idx;
      return true;
    }

    for (unsigned nbr : nhood8(idx, costmap)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}

// Translation OccupancyGrid to Costmap2D
static std::array<unsigned char, 256> initTranslationTable() {
  std::array<unsigned char, 256> cost_translation_table{};

  for (std::size_t i = 0; i < 256; ++i) {
    cost_translation_table[i] =
        static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
  }

  cost_translation_table[0] = nav2_costmap_2d::FREE_SPACE;
  cost_translation_table[99] = 253;
  cost_translation_table[100] = nav2_costmap_2d::LETHAL_OBSTACLE;
  cost_translation_table[static_cast<unsigned char>(-1)] =
      nav2_costmap_2d::NO_INFORMATION;

  return cost_translation_table;
}

void quat_to_euler(geometry_msgs::msg::Quaternion &q,
                   std::array<double, 3> &euler) {

  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  euler[0] = roll;
  euler[1] = pitch;
  euler[2] = yaw;
}

#endif