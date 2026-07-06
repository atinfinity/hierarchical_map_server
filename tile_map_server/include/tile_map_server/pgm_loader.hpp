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

/// バイナリPGM(P5, maxval<=255)を読み込み、map_server互換のトリナリ変換で
/// 占有格子 {0, 100, -1} に変換する。
/// 戻り値は行0が地図下端(OccupancyGridのデータ順)。
/// サイズが tile_size_cells 角でない・形式不正の場合は std::nullopt と
/// error にメッセージを返す。
std::optional<std::vector<int8_t>> loadTileOccupancy(
  const TilesetInfo & info, const std::filesystem::path & path, std::string & error);

}  // namespace tile_map_server

#endif  // TILE_MAP_SERVER__PGM_LOADER_HPP_
