#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "tile_map_server/tile_map_server_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tile_map_server::TileMapServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
