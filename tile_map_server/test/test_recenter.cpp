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
  info.tile_size_cells = 1000;  // 50 m square
  return info;
}

}  // namespace

// center tile (0,0): center (25,25), boundaries at x=0,50
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
  // Even past the boundary (x=50), do not switch until +5 m
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, 54.9, 25.0, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {0, 0}, 55.1, 25.0, 5.0));
  // Same in the negative direction (up to -5 m outside the boundary x=0)
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, -4.9, 25.0, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {0, 0}, -5.1, 25.0, 5.0));
}

TEST(Recenter, ChebyshevDistanceDiagonal)
{
  const auto info = makeInfo();
  // Even diagonally, the decision is made independently per x/y (Chebyshev)
  EXPECT_FALSE(shouldRecenter(info, {0, 0}, 54.9, 54.9, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {0, 0}, 54.9, 55.1, 5.0));
}

TEST(Recenter, NoChatterWhenOscillatingAcrossBoundary)
{
  const auto info = makeInfo();
  // Oscillating +/-2 m across the boundary (x=50) triggers no switch while centered on tile (0,0)
  for (double x : {48.0, 52.0, 49.0, 51.0, 50.0}) {
    EXPECT_FALSE(shouldRecenter(info, {0, 0}, x, 25.0, 5.0)) << "x=" << x;
  }
  // Seen from the new center (1,0) after switching, no switch occurs within the same band either
  for (double x : {48.0, 52.0, 49.0, 51.0}) {
    EXPECT_FALSE(shouldRecenter(info, {1, 0}, x, 25.0, 5.0)) << "x=" << x;
  }
}

TEST(Recenter, NegativeTiles)
{
  const auto info = makeInfo();
  // tile (-1,-1): center (-25,-25)
  EXPECT_FALSE(shouldRecenter(info, {-1, -1}, -25.0, -25.0, 5.0));
  EXPECT_FALSE(shouldRecenter(info, {-1, -1}, 4.9, -25.0, 5.0));
  EXPECT_TRUE(shouldRecenter(info, {-1, -1}, 5.1, -25.0, 5.0));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
