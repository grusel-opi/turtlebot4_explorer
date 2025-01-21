#ifndef TURTLEBOT4_PHOTOGRAPHER__UTIL__HPP
#define TURTLEBOT4_PHOTOGRAPHER__UTIL__HPP

#include <cmath>
#include <math.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


void quat_to_euler(const geometry_msgs::msg::Quaternion & q, std::array<double, 3> & euler) {
    
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