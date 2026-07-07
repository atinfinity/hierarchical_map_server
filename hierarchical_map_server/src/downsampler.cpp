#include "hierarchical_map_server/downsampler.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

namespace hierarchical_map_server
{

std::vector<int8_t> downsampleTile(
  const std::vector<int8_t> & hires, int tile_cells, int f, bool occupancy_priority)
{
  if (f <= 0 || tile_cells % f != 0) {
    throw std::invalid_argument("downsample factor must be > 0 and divide tile_size_cells");
  }
  if (hires.size() != static_cast<std::size_t>(tile_cells) * tile_cells) {
    throw std::invalid_argument("hires size does not match tile_cells^2");
  }

  const int lo_cells = tile_cells / f;
  std::vector<int8_t> out(static_cast<std::size_t>(lo_cells) * lo_cells, -1);

  for (int lr = 0; lr < lo_cells; ++lr) {
    for (int lc = 0; lc < lo_cells; ++lc) {
      int occupied = 0, free = 0, unknown = 0;
      for (int dr = 0; dr < f; ++dr) {
        const int hr = lr * f + dr;
        const int8_t * row = hires.data() + static_cast<std::size_t>(hr) * tile_cells + lc * f;
        for (int dc = 0; dc < f; ++dc) {
          const int8_t v = row[dc];
          if (v == 100) {
            ++occupied;
          } else if (v == 0) {
            ++free;
          } else {
            ++unknown;
          }
        }
      }

      int8_t result;
      if (occupancy_priority) {
        // Conservative: if even one cell is occupied, the result is occupied. Keeps walls.
        result = occupied > 0 ? 100 : (free > 0 ? 0 : -1);
      } else {
        // Majority vote (ties go to occupied). Unknown if there are no known cells.
        if (occupied == 0 && free == 0) {
          result = -1;
        } else {
          result = occupied >= free ? 100 : 0;
        }
      }
      out[static_cast<std::size_t>(lr) * lo_cells + lc] = result;
    }
  }
  return out;
}

std::vector<TileIndex> enumerateTiles(const std::filesystem::path & tiles_dir)
{
  std::vector<TileIndex> tiles;
  std::error_code ec;
  if (!std::filesystem::is_directory(tiles_dir, ec)) {
    return tiles;
  }
  for (const auto & entry : std::filesystem::directory_iterator(tiles_dir, ec)) {
    if (ec) {
      break;
    }
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto name = entry.path().filename().string();
    // Expected format: tile_{x}_{y}.pgm  (x,y are integers, may be negative)
    if (name.rfind("tile_", 0) != 0 || entry.path().extension() != ".pgm") {
      continue;
    }
    const std::string stem = name.substr(5, name.size() - 5 - 4);  // extract "x_y"
    const auto us = stem.find('_', stem.front() == '-' ? 1 : 0);
    if (us == std::string::npos) {
      continue;
    }
    try {
      std::size_t p1 = 0, p2 = 0;
      const int x = std::stoi(stem.substr(0, us), &p1);
      const int y = std::stoi(stem.substr(us + 1), &p2);
      if (p1 == us && p2 == stem.size() - us - 1) {
        tiles.push_back(TileIndex{x, y});
      }
    } catch (...) {
      // Ignore file names that cannot be parsed
    }
  }
  return tiles;
}

nav_msgs::msg::OccupancyGrid assembleLowresMap(
  const TilesetInfo & info,
  const std::vector<TileIndex> & tiles,
  int downsample_factor,
  bool occupancy_priority,
  const std::function<std::optional<std::vector<int8_t>>(const TileIndex &)> & load_tile)
{
  const int f = downsample_factor;
  if (f <= 0 || info.tile_size_cells % f != 0) {
    throw std::invalid_argument(
            "downsample_factor must be > 0 and divide tile_size_cells");
  }

  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = static_cast<float>(info.resolution * f);
  grid.info.origin.orientation.w = 1.0;

  if (tiles.empty()) {
    grid.info.width = 0;
    grid.info.height = 0;
    return grid;
  }

  int min_x = std::numeric_limits<int>::max(), min_y = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min(), max_y = std::numeric_limits<int>::min();
  for (const auto & t : tiles) {
    min_x = std::min(min_x, t.x);
    min_y = std::min(min_y, t.y);
    max_x = std::max(max_x, t.x);
    max_y = std::max(max_y, t.y);
  }

  const int lo_tile = info.tile_size_cells / f;              // side of a low-resolution tile
  const int grid_w = (max_x - min_x + 1) * lo_tile;
  const int grid_h = (max_y - min_y + 1) * lo_tile;

  grid.info.width = static_cast<uint32_t>(grid_w);
  grid.info.height = static_cast<uint32_t>(grid_h);
  grid.info.origin.position.x = info.origin_x + min_x * info.tile_size_m();
  grid.info.origin.position.y = info.origin_y + min_y * info.tile_size_m();
  grid.data.assign(static_cast<std::size_t>(grid_w) * grid_h, -1);

  for (const auto & t : tiles) {
    auto hires = load_tile(t);
    if (!hires) {
      continue;  // missing/corrupt tiles remain unknown
    }
    const auto lo = downsampleTile(*hires, info.tile_size_cells, f, occupancy_priority);
    const int col0 = (t.x - min_x) * lo_tile;
    const int row0 = (t.y - min_y) * lo_tile;
    for (int r = 0; r < lo_tile; ++r) {
      const std::size_t dst = static_cast<std::size_t>(row0 + r) * grid_w + col0;
      std::copy(
        lo.begin() + static_cast<std::ptrdiff_t>(r) * lo_tile,
        lo.begin() + static_cast<std::ptrdiff_t>(r + 1) * lo_tile,
        grid.data.begin() + static_cast<std::ptrdiff_t>(dst));
    }
  }
  return grid;
}

}  // namespace hierarchical_map_server
