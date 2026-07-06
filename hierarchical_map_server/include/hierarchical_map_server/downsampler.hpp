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

/// タイルの高解像度占有格子(行0=下端, 値 {0,100,-1})を係数 f で縮小する。
/// 低解像度1セルは f×f ブロックから決定:
///   occupancy_priority=true(推奨・保守的): 占有>自由>未知 の優先で、
///     ブロック内に占有が1つでもあれば占有。壁が平均化で消えない。
///   occupancy_priority=false: 多数決(同数は占有優先)。
/// hires.size() は tile_cells*tile_cells、tile_cells は f の倍数であること。
std::vector<int8_t> downsampleTile(
  const std::vector<int8_t> & hires, int tile_cells, int f, bool occupancy_priority);

/// 存在するタイルのインデックスを tiles ディレクトリから列挙する
/// (ファイル名 tile_{x}_{y}.pgm を解析)。
std::vector<TileIndex> enumerateTiles(const std::filesystem::path & tiles_dir);

/// タイル集合をダウンサンプルして低解像度の全域 OccupancyGrid を組み立てる。
/// load_tile はタイルの高解像度占有を返すコールバック(欠損は nullopt)。
/// テスト容易性のためタイル読込を注入する。header は呼び出し側で設定。
/// tiles が空の場合は width=height=0 の空グリッドを返す。
nav_msgs::msg::OccupancyGrid assembleLowresMap(
  const TilesetInfo & info,
  const std::vector<TileIndex> & tiles,
  int downsample_factor,
  bool occupancy_priority,
  const std::function<std::optional<std::vector<int8_t>>(const TileIndex &)> & load_tile);

}  // namespace hierarchical_map_server

#endif  // HIERARCHICAL_MAP_SERVER__DOWNSAMPLER_HPP_
