#include <gtest/gtest.h>

#include <vector>

#include "hierarchical_map_server/downsampler.hpp"

using hierarchical_map_server::downsampleTile;

TEST(Downsampler, OccupancyPriorityKeepsWalls)
{
  // Only one occupied cell (100) in the bottom-left block. Conservative mode keeps the low-res cell occupied.
  std::vector<int8_t> hires = {
    0, 0, 0, 0,      // row0 (bottom)
    100, 0, 0, 0,    // row1  -> bottom-left block (row0-1,col0-1) contains a 100
    0, 0, 0, 0,      // row2
    0, 0, 0, 0,      // row3 (top)
  };
  auto lo = downsampleTile(hires, 4, 2, /*occupancy_priority=*/true);
  ASSERT_EQ(lo.size(), 4u);  // 2x2
  EXPECT_EQ(lo[0], 100);     // bottom-left: occupied is kept
  EXPECT_EQ(lo[1], 0);       // bottom-right: all free
  EXPECT_EQ(lo[2], 0);       // top-left
  EXPECT_EQ(lo[3], 0);       // top-right
}

TEST(Downsampler, MajorityVoteModeCanDropSingleOccupied)
{
  // Bottom-left block has 1 occupied and 3 free -> majority vote yields free
  std::vector<int8_t> hires = {
    0, 0, 0, 0,
    100, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
  };
  auto lo = downsampleTile(hires, 4, 2, /*occupancy_priority=*/false);
  EXPECT_EQ(lo[0], 0);  // 1 vs 3 -> free
}

TEST(Downsampler, MajorityVoteTieGoesOccupied)
{
  // Bottom-left block has 2 occupied and 2 free -> ties go to occupied
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
  // Block: 1 free and 3 unknown -> free (known takes priority, no occupied)
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
