#include <gtest/gtest.h>

#include "tile_map_server/tileset.hpp"

using tile_map_server::TileIndex;
using tile_map_server::TilesetInfo;
using tile_map_server::tileCenterWorld;
using tile_map_server::tileIndexOf;

namespace
{

TilesetInfo makeInfo(double origin_x = 0.0, double origin_y = 0.0)
{
  TilesetInfo info;
  info.resolution = 0.05;
  info.tile_size_cells = 1000;  // 50 m角
  info.origin_x = origin_x;
  info.origin_y = origin_y;
  return info;
}

}  // namespace

TEST(Tileset, TileSizeMeters)
{
  EXPECT_DOUBLE_EQ(makeInfo().tile_size_m(), 50.0);
}

TEST(Tileset, IndexOfPositiveCoords)
{
  const auto info = makeInfo();
  EXPECT_EQ(tileIndexOf(info, 0.0, 0.0), (TileIndex{0, 0}));
  EXPECT_EQ(tileIndexOf(info, 49.99, 49.99), (TileIndex{0, 0}));
  EXPECT_EQ(tileIndexOf(info, 50.0, 0.0), (TileIndex{1, 0}));
  EXPECT_EQ(tileIndexOf(info, 250.0, 499.9), (TileIndex{5, 9}));
}

TEST(Tileset, IndexOfNegativeCoordsUsesFloor)
{
  const auto info = makeInfo();
  EXPECT_EQ(tileIndexOf(info, -0.01, -0.01), (TileIndex{-1, -1}));
  EXPECT_EQ(tileIndexOf(info, -50.0, -0.01), (TileIndex{-1, -1}));
  EXPECT_EQ(tileIndexOf(info, -50.01, -100.01), (TileIndex{-2, -3}));
}

TEST(Tileset, IndexOfWithOffsetOrigin)
{
  const auto info = makeInfo(-100.0, -150.0);
  EXPECT_EQ(tileIndexOf(info, -100.0, -150.0), (TileIndex{0, 0}));
  EXPECT_EQ(tileIndexOf(info, 0.0, 0.0), (TileIndex{2, 3}));
  EXPECT_EQ(tileIndexOf(info, -100.01, -150.01), (TileIndex{-1, -1}));
}

TEST(Tileset, TileCenterWorld)
{
  const auto info = makeInfo(-100.0, 0.0);
  double cx = 0.0, cy = 0.0;
  tileCenterWorld(info, TileIndex{0, 0}, cx, cy);
  EXPECT_DOUBLE_EQ(cx, -75.0);
  EXPECT_DOUBLE_EQ(cy, 25.0);
  tileCenterWorld(info, TileIndex{-1, 2}, cx, cy);
  EXPECT_DOUBLE_EQ(cx, -125.0);
  EXPECT_DOUBLE_EQ(cy, 125.0);
}

TEST(Tileset, TilePathNaming)
{
  auto info = makeInfo();
  info.tiles_dir = "/data/tiles";
  EXPECT_EQ(info.tilePath({3, -2}).string(), "/data/tiles/tile_3_-2.pgm");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
