#include <rclcpp_components/register_node_macro.hpp>
#include <costmap_to_point/costmap_to_point_component.hpp>


namespace costmap_to_point
{
CostmapToPointComponent::CostmapToPointComponent(const rclcpp::NodeOptions & options)
: Node("costmap_to_point", options)
{
  //param
  std::string grid_map_topic;
  std::string current_pose_topic_;
  declare_parameter<std::string>("grid_map_topic","/perception/grid_map");
  get_parameter("grid_map_topic", grid_map_topic);
  declare_parameter<std::string>("current_pose_topic","/current_pose");
  get_parameter("current_pose_topic",current_pose_topic_);

  //publisher
  ideal_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("ideal_scan", 1);

  //subscriber
  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic, 1,
    std::bind(&CostmapToPointComponent::GridmapCallback, this, std::placeholders::_1));
  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic_, 1,
    std::bind(&CostmapToPointComponent::CurrentPoseCallback, this, std::placeholders::_1));
}

void CostmapToPointComponent::GridmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, input_map);
  grid_map::Index current_pos(current_pose_.position.x,current_pose_.position.y);
  double search_radius_grid = serch_range_radius / input_map.getLength().x();
  const double resolution = 1.0 * M_PI / 180.0; 
  
  auto ideal_laser_scan = sensor_msgs::msg::LaserScan();
  ideal_laser_scan.header.stamp = this->now();
  ideal_laser_scan.header.frame_id = "base_link";
  ideal_laser_scan.angle_min = 0.0;
  ideal_laser_scan.angle_max = 2 * M_PI;
  ideal_laser_scan.angle_increment = resolution;
  ideal_laser_scan.range_min = 0.0;
  ideal_laser_scan.range_max = search_radius_grid * input_map.getLength().x();
  
  for (double angle = 0.0; angle < 2 * M_PI; angle += resolution) {
    grid_map::Index search_end(current_pos.x() + search_radius_grid * std::cos(angle), current_pos.y() + search_radius_grid * std::sin(angle));
    double dist = 0.0 ;
    for (grid_map::LineIterator iterator(input_map,current_pos,search_end);!iterator.isPastEnd(); ++iterator)
    {
      if (input_map.at("layer", *iterator) >= 0.8) {
        dist = std::hypot((*iterator).x() - current_pos.x(), (*iterator).y() - current_pos.y()); 
      }
    }
    ideal_laser_scan.ranges.push_back(dist);
  }
  ideal_scan_pub_->publish(ideal_laser_scan);
}

void CostmapToPointComponent::CurrentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = msg->pose;
}
}  // namespace costmap_to_point
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_to_point::CostmapToPointComponent)

