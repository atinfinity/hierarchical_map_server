#include "tile_map_server/tileset.hpp"

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tile_map_server
{

TilesetInfo loadTilesetInfo(const std::filesystem::path & tileset_yaml)
{
  if (!std::filesystem::exists(tileset_yaml)) {
    throw std::runtime_error("tileset.yaml not found: " + tileset_yaml.string());
  }

  YAML::Node doc = YAML::LoadFile(tileset_yaml.string());

  TilesetInfo info;
  try {
    info.resolution = doc["resolution"].as<double>();
    info.tile_size_cells = doc["tile_size_cells"].as<int>();
    auto origin = doc["origin"];
    info.origin_x = origin[0].as<double>();
    info.origin_y = origin[1].as<double>();
    if (doc["negate"]) {
      info.negate = doc["negate"].as<int>() != 0;
    }
    if (doc["occupied_thresh"]) {
      info.occupied_thresh = doc["occupied_thresh"].as<double>();
    }
    if (doc["free_thresh"]) {
      info.free_thresh = doc["free_thresh"].as<double>();
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
            "invalid tileset.yaml (" + tileset_yaml.string() + "): " + e.what());
  }

  if (info.free_thresh >= info.occupied_thresh) {
    throw std::runtime_error("tileset.yaml: free_thresh must be < occupied_thresh");
  }
  if (info.resolution <= 0.0) {
    throw std::runtime_error("tileset.yaml: resolution must be > 0");
  }
  if (info.tile_size_cells <= 0) {
    throw std::runtime_error("tileset.yaml: tile_size_cells must be > 0");
  }

  info.tiles_dir = tileset_yaml.parent_path() / "tiles";
  if (!std::filesystem::is_directory(info.tiles_dir)) {
    throw std::runtime_error("tiles directory not found: " + info.tiles_dir.string());
  }
  return info;
}

}  // namespace tile_map_server
