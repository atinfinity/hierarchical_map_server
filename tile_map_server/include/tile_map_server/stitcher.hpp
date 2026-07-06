#ifndef TILE_MAP_SERVER__STITCHER_HPP_
#define TILE_MAP_SERVER__STITCHER_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "tile_map_server/tile_cache.hpp"
#include "tile_map_server/tileset.hpp"

namespace tile_map_server
{

/// center を中心とする window_size×window_size タイルを1枚の
/// OccupancyGrid に結合する。欠損タイルは未知(-1)のまま。
/// header(frame_id, stamp)は呼び出し側で設定すること。
/// window_size は正の奇数であること。
nav_msgs::msg::OccupancyGrid stitchWindow(
  const TilesetInfo & info, TileCache & cache, const TileIndex & center, int window_size);

}  // namespace tile_map_server

#endif  // TILE_MAP_SERVER__STITCHER_HPP_
