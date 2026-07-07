#ifndef HIERARCHICAL_MAP_SERVER__GLOBAL_LOWRES_MAP_SERVER_NODE_HPP_
#define HIERARCHICAL_MAP_SERVER__GLOBAL_LOWRES_MAP_SERVER_NODE_HPP_

#include <memory>
#include <string>

#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tile_map_server/tileset.hpp"

namespace hierarchical_map_server
{

/// A LifecycleNode that, at startup, downsamples the tile dataset to generate a
/// single low-resolution global map and publishes it latched. For the global
/// costmap's static layer.
class GlobalLowresMapServer : public nav2_util::LifecycleNode
{
public:
  explicit GlobalLowresMapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  std::string tileset_path_;
  int downsample_factor_{4};
  bool occupancy_priority_{true};
  std::string topic_name_{"map_global_lowres"};
  std::string global_frame_{"map"};

  tile_map_server::TilesetInfo tileset_;
  nav_msgs::msg::OccupancyGrid map_;  // low-resolution global map generated at startup

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

}  // namespace hierarchical_map_server

#endif  // HIERARCHICAL_MAP_SERVER__GLOBAL_LOWRES_MAP_SERVER_NODE_HPP_
