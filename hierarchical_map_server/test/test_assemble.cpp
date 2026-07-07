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
  info.tile_size_cells = 4;   // small tile for testing
  info.origin_x = -10.0;
  info.origin_y = 20.0;
  return info;
}

// Tile with the same value in every cell
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
  // tiles (1,1) and (2,3) -> bbox x:1..2, y:1..3
  std::vector<TileIndex> tiles = {{1, 1}, {2, 3}};
  auto grid = assembleLowresMap(info, tiles, 2, true,
      [](const TileIndex &) {return std::optional<std::vector<int8_t>>(uniform(0));});

  // low-resolution tile side = 4/2 = 2. bbox 2 cols x 3 rows.
  EXPECT_EQ(grid.info.width, 2u * 2);   // (2-1+1)*2 = 4
  EXPECT_EQ(grid.info.height, 3u * 2);  // (3-1+1)*2 = 6
  EXPECT_FLOAT_EQ(grid.info.resolution, 0.1f);
  // origin = tileset origin + min_tile * tile_size_m. tile_size_m = 4*0.05 = 0.2
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -10.0 + 1 * 0.2);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 20.0 + 1 * 0.2);
  EXPECT_DOUBLE_EQ(grid.info.origin.orientation.w, 1.0);
}

TEST(Assemble, TilesPlacedAtCorrectBlocksMissingAreUnknown)
{
  auto info = makeInfo();
  // (0,0)=occupied, (1,1)=free. (1,0) and (0,1) are missing -> unknown. bbox x:0..1 y:0..1
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

  // bottom-left block (0,0) = occupied
  EXPECT_EQ(at(0, 0), 100);
  EXPECT_EQ(at(1, 1), 100);
  // top-right block (1,1) = free
  EXPECT_EQ(at(2, 2), 0);
  EXPECT_EQ(at(3, 3), 0);
  // bottom-right block (1,0) missing = unknown
  EXPECT_EQ(at(2, 0), -1);
  // top-left block (0,1) missing = unknown
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
