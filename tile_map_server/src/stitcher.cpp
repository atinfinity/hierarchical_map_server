#include "tile_map_server/stitcher.hpp"

#include <cstring>
#include <stdexcept>

namespace tile_map_server
{

nav_msgs::msg::OccupancyGrid stitchWindow(
  const TilesetInfo & info, TileCache & cache, const TileIndex & center, int window_size)
{
  if (window_size < 1 || window_size % 2 == 0) {
    throw std::invalid_argument("window_size must be a positive odd number");
  }

  const int half = window_size / 2;
  const int tile_cells = info.tile_size_cells;
  const int grid_cells = window_size * tile_cells;
  const TileIndex min_tile{center.x - half, center.y - half};

  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = static_cast<float>(info.resolution);
  grid.info.width = static_cast<uint32_t>(grid_cells);
  grid.info.height = static_cast<uint32_t>(grid_cells);
  grid.info.origin.position.x = info.origin_x + min_tile.x * info.tile_size_m();
  grid.info.origin.position.y = info.origin_y + min_tile.y * info.tile_size_m();
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(grid_cells) * grid_cells, -1);

  for (int ty = 0; ty < window_size; ++ty) {
    for (int tx = 0; tx < window_size; ++tx) {
      const auto tile = cache.get(TileIndex{min_tile.x + tx, min_tile.y + ty});
      if (!tile) {
        continue;  // missing tile stays unknown
      }
      for (int row = 0; row < tile_cells; ++row) {
        const std::size_t dst_offset =
          (static_cast<std::size_t>(ty) * tile_cells + row) * grid_cells +
          static_cast<std::size_t>(tx) * tile_cells;
        std::memcpy(
          grid.data.data() + dst_offset,
          tile->data() + static_cast<std::size_t>(row) * tile_cells,
          static_cast<std::size_t>(tile_cells));
      }
    }
  }
  return grid;
}

}  // namespace tile_map_server
