#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hierarchical_map_server/global_lowres_map_server_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hierarchical_map_server::GlobalLowresMapServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
