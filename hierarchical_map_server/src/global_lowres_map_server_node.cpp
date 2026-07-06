#include "hierarchical_map_server/global_lowres_map_server_node.hpp"

#include <chrono>
#include <string>

#include "hierarchical_map_server/downsampler.hpp"
#include "tile_map_server/pgm_loader.hpp"

namespace hierarchical_map_server
{

GlobalLowresMapServer::GlobalLowresMapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("global_lowres_map_server", "", options)
{
  declare_parameter("tileset_path", std::string(""));
  declare_parameter("downsample_factor", 4);
  declare_parameter("occupancy_priority", true);
  declare_parameter("topic_name", std::string("map_global_lowres"));
  declare_parameter("global_frame", std::string("map"));
}

nav2_util::CallbackReturn GlobalLowresMapServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tileset_path_ = get_parameter("tileset_path").as_string();
  downsample_factor_ = static_cast<int>(get_parameter("downsample_factor").as_int());
  occupancy_priority_ = get_parameter("occupancy_priority").as_bool();
  topic_name_ = get_parameter("topic_name").as_string();
  global_frame_ = get_parameter("global_frame").as_string();

  try {
    tileset_ = tile_map_server::loadTilesetInfo(tileset_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load tileset: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (downsample_factor_ <= 0 || tileset_.tile_size_cells % downsample_factor_ != 0) {
    RCLCPP_ERROR(
      get_logger(),
      "downsample_factor (%d) must be > 0 and divide tile_size_cells (%d)",
      downsample_factor_, tileset_.tile_size_cells);
    return nav2_util::CallbackReturn::FAILURE;
  }

  const auto tiles = enumerateTiles(tileset_.tiles_dir);
  if (tiles.empty()) {
    RCLCPP_ERROR(get_logger(), "No tiles found in %s", tileset_.tiles_dir.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  const auto t0 = std::chrono::steady_clock::now();
  try {
    map_ = assembleLowresMap(
      tileset_, tiles, downsample_factor_, occupancy_priority_,
      [this](const TileIndex & idx) -> std::optional<std::vector<int8_t>> {
        std::string error;
        auto occ = tile_map_server::loadTileOccupancy(tileset_, tileset_.tilePath(idx), error);
        if (!occ) {
          RCLCPP_WARN(get_logger(), "Tile load error: %s", error.c_str());
        }
        return occ;
      });
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to assemble low-res map: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t0).count();

  map_.header.frame_id = global_frame_;

  RCLCPP_INFO(
    get_logger(),
    "Built low-res map: %u x %u @ %.3f m/cell (%.1f x %.1f m) from %zu tiles "
    "(factor %d) in %ld ms",
    map_.info.width, map_.info.height, map_.info.resolution,
    map_.info.width * map_.info.resolution, map_.info.height * map_.info.resolution,
    tiles.size(), downsample_factor_, elapsed_ms);

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GlobalLowresMapServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  map_pub_->on_activate();
  map_.header.stamp = now();
  map_.info.map_load_time = map_.header.stamp;
  map_pub_->publish(map_);
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GlobalLowresMapServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  map_pub_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GlobalLowresMapServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  map_pub_.reset();
  map_ = nav_msgs::msg::OccupancyGrid();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GlobalLowresMapServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace hierarchical_map_server
