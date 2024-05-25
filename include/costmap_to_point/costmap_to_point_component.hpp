#ifndef COSTMAP_TO_POINT_HPP_
#define COSTMAP_TO_POINT_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <bits/stdc++.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace costmap_to_point
{
class CostmapToPointComponent : public rclcpp::Node
{
public:
  explicit CostmapToPointComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr ideal_scan_pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  void gridmap_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  geometry_msgs::msg::Pose current_pose_;
  int serch_range_radius = 50; 
};
}  // namespace costmap_to_point
#endif