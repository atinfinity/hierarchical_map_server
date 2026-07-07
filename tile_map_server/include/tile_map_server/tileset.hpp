#ifndef TILE_MAP_SERVER__TILESET_HPP_
#define TILE_MAP_SERVER__TILESET_HPP_

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>

namespace tile_map_server
{

struct TileIndex
{
  int x{0};
  int y{0};
  bool operator==(const TileIndex & other) const {return x == other.x && y == other.y;}
  bool operator!=(const TileIndex & other) const {return !(*this == other);}
};

struct TileIndexHash
{
  std::size_t operator()(const TileIndex & t) const
  {
    return std::hash<int64_t>()(
      (static_cast<int64_t>(t.x) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(t.y)));
  }
};

/// Contents of tileset.yaml. Resolution, size, and thresholds are shared by all tiles.
struct TilesetInfo
{
  double resolution{0.05};       // [m/cell]
  int tile_size_cells{1000};     // number of cells per tile edge
  double origin_x{0.0};          // global coordinates of the lower-left corner of tile (0,0) [m]
  double origin_y{0.0};
  bool negate{false};
  // map_server standard defaults. free_thresh is 0.196 so that the PGM unknown color 205
  // (occ=50/255≈0.1961) stays within the unknown band.
  double occupied_thresh{0.65};
  double free_thresh{0.196};
  std::filesystem::path tiles_dir;

  double tile_size_m() const {return resolution * tile_size_cells;}

  std::filesystem::path tilePath(const TileIndex & idx) const
  {
    return tiles_dir / ("tile_" + std::to_string(idx.x) + "_" + std::to_string(idx.y) + ".pgm");
  }
};

/// Load tileset.yaml. Throws std::runtime_error on invalid contents.
TilesetInfo loadTilesetInfo(const std::filesystem::path & tileset_yaml);

/// Index of the tile that the world coordinates belong to (floor division to handle negative coordinates)
inline TileIndex tileIndexOf(const TilesetInfo & info, double wx, double wy)
{
  const double s = info.tile_size_m();
  return TileIndex{
    static_cast<int>(std::floor((wx - info.origin_x) / s)),
    static_cast<int>(std::floor((wy - info.origin_y) / s))};
}

/// World coordinates of the tile center
inline void tileCenterWorld(const TilesetInfo & info, const TileIndex & idx, double & cx, double & cy)
{
  const double s = info.tile_size_m();
  cx = info.origin_x + (idx.x + 0.5) * s;
  cy = info.origin_y + (idx.y + 0.5) * s;
}

/// Recenter decision. Returns true only when the Chebyshev distance from the
/// center of the current center tile exceeds "half tile edge + hysteresis".
inline bool shouldRecenter(
  const TilesetInfo & info, const TileIndex & center, double wx, double wy, double hysteresis_m)
{
  double cx = 0.0, cy = 0.0;
  tileCenterWorld(info, center, cx, cy);
  const double d = std::max(std::abs(wx - cx), std::abs(wy - cy));
  return d > info.tile_size_m() / 2.0 + hysteresis_m;
}

}  // namespace tile_map_server

#endif  // TILE_MAP_SERVER__TILESET_HPP_
