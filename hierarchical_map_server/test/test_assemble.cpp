#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <vector>

#include "hierarchical_map_server/downsampler.hpp"

using hierarchical_map_server::TileIndex;
using hierarchical_map_server::TilesetInfo;
using hierarchical_map_server::assembleLowresMap;

namespace
{

TilesetInfo makeInfo()
{
  TilesetInfo info;
  info.resolution = 0.05;
  info.tile_size_cells = 4;   // テスト用に小さいタイル
  info.origin_x = -10.0;
  info.origin_y = 20.0;
  return info;
}

// 全セル同一値のタイル
std::vector<int8_t> uniform(int8_t v) {return std::vector<int8_t>(16, v); }

}  // namespace

TEST(Assemble, EmptyTilesGivesEmptyGrid)
{
  auto info = makeInfo();
  auto grid = assembleLowresMap(info, {}, 2, true,
      [](const TileIndex &) -> std::optional<std::vector<int8_t>> {return std::nullopt;});
  EXPECT_EQ(grid.info.width, 0u);
  EXPECT_EQ(grid.info.height, 0u);
  EXPECT_FLOAT_EQ(grid.info.resolution, 0.1f);  // 0.05 * 2
}

TEST(Assemble, GeometryAndOrigin)
{
  auto info = makeInfo();
  // タイル (1,1) と (2,3) → bbox x:1..2, y:1..3
  std::vector<TileIndex> tiles = {{1, 1}, {2, 3}};
  auto grid = assembleLowresMap(info, tiles, 2, true,
      [](const TileIndex &) {return std::optional<std::vector<int8_t>>(uniform(0));});

  // 低解像度タイル一辺 = 4/2 = 2。bbox 2列 x 3行。
  EXPECT_EQ(grid.info.width, 2u * 2);   // (2-1+1)*2 = 4
  EXPECT_EQ(grid.info.height, 3u * 2);  // (3-1+1)*2 = 6
  EXPECT_FLOAT_EQ(grid.info.resolution, 0.1f);
  // origin = tileset原点 + min_tile * tile_size_m。tile_size_m = 4*0.05 = 0.2
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -10.0 + 1 * 0.2);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 20.0 + 1 * 0.2);
  EXPECT_DOUBLE_EQ(grid.info.origin.orientation.w, 1.0);
}

TEST(Assemble, TilesPlacedAtCorrectBlocksMissingAreUnknown)
{
  auto info = makeInfo();
  // (0,0)=占有, (1,1)=自由。(1,0)と(0,1)は欠損 → 未知。bbox x:0..1 y:0..1
  std::map<std::pair<int, int>, int8_t> present = {{{0, 0}, 100}, {{1, 1}, 0}};
  std::vector<TileIndex> tiles = {{0, 0}, {1, 1}};

  auto grid = assembleLowresMap(info, tiles, 2, true,
      [&](const TileIndex & t) -> std::optional<std::vector<int8_t>> {
        auto it = present.find({t.x, t.y});
        if (it == present.end()) {return std::nullopt;}
        return uniform(it->second);
      });

  const int W = 4;  // 2 tiles * 2
  auto at = [&](int col, int row) {return grid.data[row * W + col];};
  EXPECT_EQ(grid.info.width, 4u);
  EXPECT_EQ(grid.info.height, 4u);

  // 左下ブロック(0,0) = 占有
  EXPECT_EQ(at(0, 0), 100);
  EXPECT_EQ(at(1, 1), 100);
  // 右上ブロック(1,1) = 自由
  EXPECT_EQ(at(2, 2), 0);
  EXPECT_EQ(at(3, 3), 0);
  // 右下ブロック(1,0) 欠損 = 未知
  EXPECT_EQ(at(2, 0), -1);
  // 左上ブロック(0,1) 欠損 = 未知
  EXPECT_EQ(at(0, 2), -1);
}

TEST(Assemble, NegativeTileIndices)
{
  auto info = makeInfo();
  std::vector<TileIndex> tiles = {{-1, -2}};
  auto grid = assembleLowresMap(info, tiles, 2, true,
      [](const TileIndex &) {return std::optional<std::vector<int8_t>>(uniform(100));});
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -10.0 + (-1) * 0.2);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 20.0 + (-2) * 0.2);
  EXPECT_EQ(grid.data[0], 100);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
