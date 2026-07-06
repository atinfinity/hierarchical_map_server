#include <gtest/gtest.h>

#include "tile_map_server/tileset.hpp"

using tile_map_server::TileIndex;
using tile_map_server::TilesetInfo;
using tile_map_server::shouldRecenter;

namespace
{

TilesetInfo makeInfo()
{
  TilesetInfo info;
  info.resolution = 0.05;
  info.tile_size_cells = 1000;  // 50 m角
  return info;
}

}  // namespace

// 中心タイル(0,0): 中心(25,25)、境界はx=0,50
TEST(Recenter, StaysInsideOwnTile)
{
  const auto info = makeInfo();
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, 25.0, 25.0, 5.0));
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, 49.9, 25.0, 5.0));
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, 0.1, 0.1, 5.0));
}

TEST(Recenter, HysteresisBandBeyondBoundary)
{
  const auto info = makeInfo();
  // 境界(x=50)を越えても+5mまでは切替えない
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, 54.9, 25.0, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {0, 0}, 55.1, 25.0, 5.0));
  // 負方向も同様(境界x=0の外側-5mまで)
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, -4.9, 25.0, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {0, 0}, -5.1, 25.0, 5.0));
}

TEST(Recenter, ChebyshevDistanceDiagonal)
{
  const auto info = makeInfo();
  // 斜め方向でもx/y独立(チェビシェフ)で判定される
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, 54.9, 54.9, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {0, 0}, 54.9, 55.1, 5.0));
}

TEST(Recenter, NoChatterWhenOscillatingAcrossBoundary)
{
  const auto info = makeInfo();
  // 境界(x=50)を±2mで往復してもタイル(0,0)中心のままなら切替は起きない
  for (double x : {48.0, 52.0, 49.0, 51.0, 50.0}) {
    EXPECT_FALSE(shouldRecenter(info, {0, 0}, x, 25.0, 5.0)) << "x=" << x;
  }
  // 切替後の新中心(1,0)から見ても、同じ帯域では切替は起きない
  for (double x : {48.0, 52.0, 49.0, 51.0}) {
    EXPECT_FALSE(shouldRecenter(info, {1, 0}, x, 25.0, 5.0)) << "x=" << x;
  }
}

TEST(Recenter, NegativeTiles)
{
  const auto info = makeInfo();
  // タイル(-1,-1): 中心(-25,-25)
  EXPECT_FALSE(shouldRecenter(info, {-1, -1}, -25.0, -25.0, 5.0));
  EXPECT_FALSE(shouldRecenter(info, {-1, -1}, 4.9, -25.0, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {-1, -1}, 5.1, -25.0, 5.0));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
