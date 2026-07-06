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

/// tileset.yaml の内容。全タイルで解像度・サイズ・しきい値は共通。
struct TilesetInfo
{
  double resolution{0.05};       // [m/cell]
  int tile_size_cells{1000};     // タイル一辺のセル数
  double origin_x{0.0};          // タイル(0,0)左下隅のグローバル座標 [m]
  double origin_y{0.0};
  bool negate{false};
  // map_server標準デフォルト。free_threshが0.196なのはPGM未知色205
  // (occ=50/255≈0.1961)が未知帯に残るようにするため。
  double occupied_thresh{0.65};
  double free_thresh{0.196};
  std::filesystem::path tiles_dir;

  double tile_size_m() const {return resolution * tile_size_cells;}

  std::filesystem::path tilePath(const TileIndex & idx) const
  {
    return tiles_dir / ("tile_" + std::to_string(idx.x) + "_" + std::to_string(idx.y) + ".pgm");
  }
};

/// tileset.yaml を読み込む。不正な内容は std::runtime_error を投げる。
TilesetInfo loadTilesetInfo(const std::filesystem::path & tileset_yaml);

/// ワールド座標が属するタイルのインデックス(負座標対応のfloor除算)
inline TileIndex tileIndexOf(const TilesetInfo & info, double wx, double wy)
{
  const double s = info.tile_size_m();
  return TileIndex{
    static_cast<int>(std::floor((wx - info.origin_x) / s)),
    static_cast<int>(std::floor((wy - info.origin_y) / s))};
}

/// タイル中心のワールド座標
inline void tileCenterWorld(const TilesetInfo & info, const TileIndex & idx, double & cx, double & cy)
{
  const double s = info.tile_size_m();
  cx = info.origin_x + (idx.x + 0.5) * s;
  cy = info.origin_y + (idx.y + 0.5) * s;
}

/// 再センタリング判定。現在の中心タイルの中心からチェビシェフ距離で
/// 「タイル半辺 + ヒステリシス」を超えて離れたときのみ true。
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
