#include <gtest/gtest.h>

#include <vector>

#include "hierarchical_map_server/downsampler.hpp"

using hierarchical_map_server::downsampleTile;

TEST(Downsampler, OccupancyPriorityKeepsWalls)
{
  // 左下ブロックに1つだけ占有(100)。保守的なら低解像度セルは占有。
  std::vector<int8_t> hires = {
    0, 0, 0, 0,      // row0 (bottom)
    100, 0, 0, 0,    // row1  -> 左下ブロック(row0-1,col0-1)に100あり
    0, 0, 0, 0,      // row2
    0, 0, 0, 0,      // row3 (top)
  };
  auto lo = downsampleTile(hires, 4, 2, /*occupancy_priority=*/true);
  ASSERT_EQ(lo.size(), 4u);  // 2x2
  EXPECT_EQ(lo[0], 100);     // 左下: 占有が残る
  EXPECT_EQ(lo[1], 0);       // 右下: 全自由
  EXPECT_EQ(lo[2], 0);       // 左上
  EXPECT_EQ(lo[3], 0);       // 右上
}

TEST(Downsampler, MajorityVoteModeCanDropSingleOccupied)
{
  // 左下ブロックに占有1・自由3 → 多数決では自由
  std::vector<int8_t> hires = {
    0, 0, 0, 0,
    100, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
  };
  auto lo = downsampleTile(hires, 4, 2, /*occupancy_priority=*/false);
  EXPECT_EQ(lo[0], 0);  // 1 vs 3 → 自由
}

TEST(Downsampler, MajorityVoteTieGoesOccupied)
{
  // 左下ブロックに占有2・自由2 → 同数は占有優先
  std::vector<int8_t> hires = {
    100, 100, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
  };
  auto lo = downsampleTile(hires, 4, 2, false);
  EXPECT_EQ(lo[0], 100);
}

TEST(Downsampler, UnknownPropagatesWhenNoKnownCells)
{
  std::vector<int8_t> hires(16, -1);
  auto lo = downsampleTile(hires, 4, 2, true);
  for (int8_t v : lo) {
    EXPECT_EQ(v, -1);
  }
}

TEST(Downsampler, FreeBeatsUnknownButNotOccupied)
{
  // ブロック: 自由1・未知3 → 自由(既知が優先、占有はなし)
  std::vector<int8_t> hires = {
    0, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
  };
  auto lo = downsampleTile(hires, 4, 2, true);
  EXPECT_EQ(lo[0], 0);
  EXPECT_EQ(lo[1], -1);
  EXPECT_EQ(lo[2], -1);
  EXPECT_EQ(lo[3], -1);
}

TEST(Downsampler, FactorOneIsIdentity)
{
  std::vector<int8_t> hires = {100, 0, -1, 0};
  auto lo = downsampleTile(hires, 2, 1, true);
  EXPECT_EQ(lo, hires);
}

TEST(Downsampler, RejectsNonDivisibleFactor)
{
  std::vector<int8_t> hires(16, 0);
  EXPECT_THROW(downsampleTile(hires, 4, 3, true), std::invalid_argument);
}

TEST(Downsampler, RejectsSizeMismatch)
{
  std::vector<int8_t> hires(10, 0);
  EXPECT_THROW(downsampleTile(hires, 4, 2, true), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
