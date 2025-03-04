cmake_minimum_required(VERSION 3.8)
project(turtlebot4_explorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(deps
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        nav2_msgs
        nav2_costmap_2d
        tf2
        tf2_ros
        rclcpp_action
        nav2_map_server
        slam_toolbox
        visualization_msgs
        nav2_util
        std_msgs
        angles
        tf2_geometry_msgs
        map_msgs
        irobot_create_msgs)

set(targets
        explorer)

foreach (d ${deps})
  find_package(${d} REQUIRED)
endforeach ()

foreach (t ${targets})
  add_executable(${t} src/${t}.cpp)
  ament_target_dependencies(${t} ${deps})
endforeach ()

target_include_directories(explorer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(explorer PUBLIC c_std_99 cxx_std_17)

install(TARGETS
        ${targets}
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
