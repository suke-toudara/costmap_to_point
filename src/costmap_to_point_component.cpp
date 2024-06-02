// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <costmap_to_point/costmap_to_point_component.hpp>

namespace costmap_to_point {
CostmapToPointComponent::CostmapToPointComponent(const rclcpp::NodeOptions & options)
  : Node("costmap_to_point", options)
{
  // param
  std::string grid_map_topic;
  std::string current_pose_topic_;
  declare_parameter<std::string>("grid_map_topic", "/grid_map");
  get_parameter("grid_map_topic", grid_map_topic);
  declare_parameter<std::string>("current_pose_topic", "/current_pose");
  get_parameter("current_pose_topic", current_pose_topic_);

  // publisher
  ideal_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("ideal_scan", 1);

  // subscriber
  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
    "/perception/grid_map", 10, [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
      gridmap_callback(msg);
    });
  
  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/current_pose", 10, [this](const  geometry_msgs::msg::PoseStamped::SharedPtr  msg) {
      current_pose_callback(msg);
    });
}

void CostmapToPointComponent::gridmap_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, input_map);
  grid_map::Index current_pos(current_pose_.position.x, current_pose_.position.y);
  double search_radius_grid = serch_range_radius / input_map.getLength().x();
  const double resolution = 1.0 * M_PI / 180.0;
  const int num_ranges = static_cast<int>(2 * M_PI / resolution);

  auto ideal_laser_scan = sensor_msgs::msg::LaserScan();
  ideal_laser_scan.header.stamp = this->now();
  ideal_laser_scan.header.frame_id = "base_link";
  ideal_laser_scan.angle_min = 0.0;
  ideal_laser_scan.angle_max = 2 * M_PI;
  ideal_laser_scan.angle_increment = resolution;
  ideal_laser_scan.range_min = 0.0;
  ideal_laser_scan.range_max = 100.0; //search_radius_grid * input_map.getLength().x();
  ideal_laser_scan.ranges.resize(num_ranges);
  ideal_laser_scan.intensities.resize(num_ranges, 100.0);
  for (int i = 0; i < num_ranges; ++i) {
    double angle = i * resolution;
    grid_map::Index search_end(current_pos.x() + search_radius_grid * std::cos(angle),
                                current_pos.y() + search_radius_grid * std::sin(angle));
    double dist = 0.0;
    for (grid_map::LineIterator iterator(input_map, current_pos, search_end); !iterator.isPastEnd(); ++iterator) {
      if (input_map.at("combined", *iterator) <= 0.5) {
      }else{
        // dist = std::hypot((*iterator).x() - current_pos.x(), (*iterator).y() - current_pos.y())*resolution;
        dist = std::hypot((*iterator).x()*input_map.getResolution(),(*iterator).y()*input_map.getResolution());
        RCLCPP_INFO(get_logger(),"dist_x: %d",(*iterator).x());
        RCLCPP_INFO(get_logger(),"dist_y: %d",(*iterator).y());
        RCLCPP_INFO(get_logger(),"getResolution: %f",input_map.getResolution());
      }
    }
    ideal_laser_scan.ranges[i] = dist;
    ideal_laser_scan.intensities[i] = 100.0;
  }
  ideal_scan_pub_->publish(ideal_laser_scan);
}

void CostmapToPointComponent::current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = msg->pose;
}
}  // namespace costmap_to_point

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_to_point::CostmapToPointComponent)