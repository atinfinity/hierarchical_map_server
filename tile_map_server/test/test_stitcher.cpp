#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "tile_map_server/pgm_loader.hpp"
#include "tile_map_server/stitcher.hpp"
#include "tile_map_server/tile_cache.hpp"
#include "tile_map_server/tileset.hpp"

namespace fs = std::filesystem;
using tile_map_server::TileCache;
using tile_map_server::TileIndex;
using tile_map_server::TilesetInfo;
using tile_map_server::loadTileOccupancy;
using tile_map_server::stitchWindow;

namespace
{

constexpr int kTileCells = 4;  // small tile for testing
constexpr uint8_t kPgmOccupied = 0;
constexpr uint8_t kPgmFree = 254;
constexpr uint8_t kPgmUnknown = 205;

class StitcherTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() /
      ("tile_map_server_test_" + std::to_string(::getpid()));
    fs::create_directories(dir_ / "tiles");
    info_.resolution = 0.5;
    info_.tile_size_cells = kTileCells;  // 2 m square
    info_.origin_x = -10.0;
    info_.origin_y = 20.0;
    info_.tiles_dir = dir_ / "tiles";
  }

  void TearDown() override {fs::remove_all(dir_);}

  /// Write with row 0 = top (pixels are ordered with row 0 = top)
  void writeTile(const TileIndex & idx, const std::vector<uint8_t> & pixels)
  {
    std::ofstream f(info_.tilePath(idx), std::ios::binary);
    f << "P5\n# test tile\n" << kTileCells << " " << kTileCells << "\n255\n";
    f.write(reinterpret_cast<const char *>(pixels.data()),
      static_cast<std::streamsize>(pixels.size()));
  }

  /// Tile with all cells set to the same value
  void writeUniformTile(const TileIndex & idx, uint8_t value)
  {
    writeTile(idx, std::vector<uint8_t>(kTileCells * kTileCells, value));
  }

  fs::path dir_;
  TilesetInfo info_;
};

}  // namespace

TEST_F(StitcherTest, PgmLoaderConvertsAndFlips)
{
  // Only the top row is occupied, the rest is free
  std::vector<uint8_t> px(kTileCells * kTileCells, kPgmFree);
  for (int c = 0; c < kTileCells; ++c) {
    px[c] = kPgmOccupied;
  }
  writeTile({0, 0}, px);

  std::string err;
  auto occ = loadTileOccupancy(info_, info_.tilePath({0, 0}), err);
  ASSERT_TRUE(occ.has_value()) << err;
  // PGM top row -> last row in the OccupancyGrid (top of the map)
  const int last_row = kTileCells - 1;
  for (int c = 0; c < kTileCells; ++c) {
    EXPECT_EQ((*occ)[0 * kTileCells + c], 0);              // bottom row is free
    EXPECT_EQ((*occ)[last_row * kTileCells + c], 100);     // top row is occupied
  }
}

TEST_F(StitcherTest, PgmLoaderRejectsWrongSize)
{
  std::ofstream f(info_.tilePath({0, 0}), std::ios::binary);
  f << "P5\n2 2\n255\n";
  const char data[4] = {0, 0, 0, 0};
  f.write(data, 4);
  f.close();

  std::string err;
  EXPECT_FALSE(loadTileOccupancy(info_, info_.tilePath({0, 0}), err).has_value());
  EXPECT_NE(err.find("size mismatch"), std::string::npos);
}

TEST_F(StitcherTest, WindowGeometry)
{
  TileCache cache(info_, 16);
  const auto grid = stitchWindow(info_, cache, {1, 1}, 3);

  EXPECT_EQ(grid.info.width, 3u * kTileCells);
  EXPECT_EQ(grid.info.height, 3u * kTileCells);
  EXPECT_FLOAT_EQ(grid.info.resolution, 0.5f);
  // window bottom-left = bottom-left of tile (0,0) = tileset origin
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -10.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 20.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.orientation.w, 1.0);
}

TEST_F(StitcherTest, MissingTilesAreUnknown)
{
  TileCache cache(info_, 16);
  const auto grid = stitchWindow(info_, cache, {0, 0}, 3);
  for (int8_t v : grid.data) {
    EXPECT_EQ(v, -1);
  }
}

TEST_F(StitcherTest, TilesPlacedAtCorrectOffsets)
{
  // window center (1,1): bottom-left tile (0,0)=free, center (1,1)=occupied, top-right (2,2)=free. Others missing
  writeUniformTile({0, 0}, kPgmFree);
  writeUniformTile({1, 1}, kPgmOccupied);
  writeUniformTile({2, 2}, kPgmFree);

  TileCache cache(info_, 16);
  const auto grid = stitchWindow(info_, cache, {1, 1}, 3);
  const int W = 3 * kTileCells;
  auto at = [&](int col, int row) {return grid.data[row * W + col];};

  const int T = kTileCells;
  EXPECT_EQ(at(0, 0), 0);                    // bottom-left block = free
  EXPECT_EQ(at(T - 1, T - 1), 0);
  EXPECT_EQ(at(T, T), 100);                  // center block = occupied
  EXPECT_EQ(at(2 * T - 1, 2 * T - 1), 100);
  EXPECT_EQ(at(2 * T, 2 * T), 0);            // top-right block = free
  EXPECT_EQ(at(W - 1, W - 1), 0);
  EXPECT_EQ(at(T, 0), -1);                   // missing tile (1,0) = unknown
  EXPECT_EQ(at(0, T), -1);                   // missing tile (0,1) = unknown
}

TEST_F(StitcherTest, UnknownPixelsStayUnknown)
{
  writeUniformTile({0, 0}, kPgmUnknown);
  TileCache cache(info_, 16);
  const auto grid = stitchWindow(info_, cache, {0, 0}, 1);
  ASSERT_EQ(grid.data.size(), static_cast<std::size_t>(kTileCells * kTileCells));
  for (int8_t v : grid.data) {
    EXPECT_EQ(v, -1);
  }
}

TEST_F(StitcherTest, NegativeTileIndices)
{
  writeUniformTile({-1, -1}, kPgmOccupied);
  TileCache cache(info_, 16);
  const auto grid = stitchWindow(info_, cache, {-1, -1}, 1);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -10.0 - 2.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, 20.0 - 2.0);
  EXPECT_EQ(grid.data[0], 100);
}

TEST_F(StitcherTest, CacheEvictsLeastRecentlyUsed)
{
  writeUniformTile({0, 0}, kPgmFree);
  writeUniformTile({1, 0}, kPgmFree);
  writeUniformTile({2, 0}, kPgmFree);

  TileCache cache(info_, 2);
  EXPECT_NE(cache.get({0, 0}), nullptr);
  EXPECT_NE(cache.get({1, 0}), nullptr);
  EXPECT_EQ(cache.size(), 2u);
  cache.get({2, 0});  // (0,0) is evicted
  EXPECT_EQ(cache.size(), 2u);

  // The evicted (0,0) is reloaded from disk (after deletion it becomes nullptr = unknown)
  fs::remove(info_.tilePath({0, 0}));
  EXPECT_EQ(cache.get({0, 0}), nullptr);
}

TEST_F(StitcherTest, RejectsEvenWindowSize)
{
  TileCache cache(info_, 16);
  EXPECT_THROW(stitchWindow(info_, cache, {0, 0}, 2), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
