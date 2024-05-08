#include <memory>
#include <costmap_to_point/costmap_to_point_component.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<costmap_to_point::CostmapToPointComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}