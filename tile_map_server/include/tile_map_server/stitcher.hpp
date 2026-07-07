#ifndef TILE_MAP_SERVER__STITCHER_HPP_
#define TILE_MAP_SERVER__STITCHER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "tile_map_server/tile_cache.hpp"
#include "tile_map_server/tileset.hpp"

namespace tile_map_server
{

/// Stitch a window_size×window_size block of tiles centered on center into a
/// single OccupancyGrid. Missing tiles remain unknown (-1).
/// The header (frame_id, stamp) must be set by the caller.
/// window_size must be a positive odd number.
nav_msgs::msg::OccupancyGrid stitchWindow(
  const TilesetInfo & info, TileCache & cache, const TileIndex & center, int window_size);

}  // namespace tile_map_server

#endif  // TILE_MAP_SERVER__STITCHER_HPP_
