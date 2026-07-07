#ifndef TILE_MAP_SERVER__PGM_LOADER_HPP_
#define TILE_MAP_SERVER__PGM_LOADER_HPP_

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "tile_map_server/tileset.hpp"

namespace tile_map_server
{

/// Read a binary PGM (P5, maxval<=255) and convert it into an occupancy grid
/// {0, 100, -1} using map_server-compatible trinary conversion.
/// In the return value, row 0 is the bottom of the map (OccupancyGrid data order).
/// If the size is not a tile_size_cells square or the format is invalid, returns
/// std::nullopt and writes a message to error.
std::optional<std::vector<int8_t>> loadTileOccupancy(
  const TilesetInfo & info, const std::filesystem::path & path, std::string & error);

}  // namespace tile_map_server

#endif  // TILE_MAP_SERVER__PGM_LOADER_HPP_
