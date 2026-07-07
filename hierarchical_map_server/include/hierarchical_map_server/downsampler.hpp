#ifndef HIERARCHICAL_MAP_SERVER__DOWNSAMPLER_HPP_
#define HIERARCHICAL_MAP_SERVER__DOWNSAMPLER_HPP_

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "tile_map_server/tileset.hpp"

namespace hierarchical_map_server
{

using tile_map_server::TileIndex;
using tile_map_server::TilesetInfo;

/// Downsamples a tile's high-resolution occupancy grid (row 0 = bottom,
/// values {0,100,-1}) by factor f.
/// Each low-resolution cell is decided from an f x f block:
///   occupancy_priority=true (recommended, conservative): priority
///     occupied > free > unknown; if the block contains even one occupied
///     cell it becomes occupied. Walls are not lost through averaging.
///   occupancy_priority=false: majority vote (ties go to occupied).
/// hires.size() must be tile_cells*tile_cells, and tile_cells must be a
/// multiple of f.
std::vector<int8_t> downsampleTile(
  const std::vector<int8_t> & hires, int tile_cells, int f, bool occupancy_priority);

/// Enumerates the indices of existing tiles from the tiles directory
/// (parses file names tile_{x}_{y}.pgm).
std::vector<TileIndex> enumerateTiles(const std::filesystem::path & tiles_dir);

/// Downsamples a set of tiles and assembles a low-resolution global OccupancyGrid.
/// load_tile is a callback returning a tile's high-resolution occupancy
/// (nullopt if missing).
/// Tile loading is injected for testability; the header is set by the caller.
/// If tiles is empty, returns an empty grid with width=height=0.
nav_msgs::msg::OccupancyGrid assembleLowresMap(
  const TilesetInfo & info,
  const std::vector<TileIndex> & tiles,
  int downsample_factor,
  bool occupancy_priority,
  const std::function<std::optional<std::vector<int8_t>>(const TileIndex &)> & load_tile);

}  // namespace hierarchical_map_server

#endif  // HIERARCHICAL_MAP_SERVER__DOWNSAMPLER_HPP_
