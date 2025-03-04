cmake_minimum_required(VERSION 3.8)
project(turtlebot4_photographer)

# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)

set(deps
  rclcpp
  rclcpp_components
  nav_msgs
  nav2_msgs
  nav2_util
  tf2_ros
  nav2_core
  pluginlib
  image_transport
  cv_bridge
  OpenCV
)

foreach (d ${deps})
  find_package(${d} REQUIRED)
endforeach ()

add_library(photographer_component SHARED src/photographer.cpp)

ament_target_dependencies(photographer_component ${deps})

target_include_directories(photographer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(photographer_component PUBLIC c_std_99 cxx_std_17)

rclcpp_components_register_node(
  photographer_component
    PLUGIN "turtlebot4_photographer::Photographer"
    EXECUTABLE photographer
)
ament_export_targets(export_photographer_component)
install(TARGETS photographer_component
        EXPORT export_photographer_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
