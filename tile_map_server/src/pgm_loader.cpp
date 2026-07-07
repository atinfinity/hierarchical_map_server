#include "tile_map_server/pgm_loader.hpp"

#include <fstream>
#include <limits>

namespace tile_map_server
{

namespace
{

/// Read one token from the PGM header (skipping whitespace and '#' comments)
bool nextToken(std::istream & is, std::string & token)
{
  token.clear();
  int c = is.get();
  while (c != EOF) {
    if (c == '#') {
      while (c != EOF && c != '\n') {
        c = is.get();
      }
    } else if (std::isspace(c)) {
      c = is.get();
    } else {
      break;
    }
  }
  while (c != EOF && !std::isspace(c) && c != '#') {
    token.push_back(static_cast<char>(c));
    c = is.get();
  }
  // The one character right after the token (the delimiter) may be consumed
  return !token.empty();
}

bool parsePositiveInt(const std::string & s, int & out)
{
  try {
    std::size_t pos = 0;
    const long v = std::stol(s, &pos);
    if (pos != s.size() || v <= 0 || v > std::numeric_limits<int>::max()) {
      return false;
    }
    out = static_cast<int>(v);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

std::optional<std::vector<int8_t>> loadTileOccupancy(
  const TilesetInfo & info, const std::filesystem::path & path, std::string & error)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    error = "cannot open file: " + path.string();
    return std::nullopt;
  }

  std::string magic, w_s, h_s, maxval_s;
  if (!nextToken(file, magic) || magic != "P5") {
    error = "not a binary PGM (P5): " + path.string();
    return std::nullopt;
  }
  int width = 0, height = 0, maxval = 0;
  if (!nextToken(file, w_s) || !parsePositiveInt(w_s, width) ||
    !nextToken(file, h_s) || !parsePositiveInt(h_s, height) ||
    !nextToken(file, maxval_s) || !parsePositiveInt(maxval_s, maxval))
  {
    error = "invalid PGM header: " + path.string();
    return std::nullopt;
  }
  if (maxval > 255) {
    error = "16-bit PGM is not supported: " + path.string();
    return std::nullopt;
  }
  if (width != info.tile_size_cells || height != info.tile_size_cells) {
    error = "tile size mismatch (" + std::to_string(width) + "x" + std::to_string(height) +
      ", expected " + std::to_string(info.tile_size_cells) + "): " + path.string();
    return std::nullopt;
  }

  const std::size_t n = static_cast<std::size_t>(width) * height;
  std::vector<uint8_t> raw(n);
  file.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(n));
  if (static_cast<std::size_t>(file.gcount()) != n) {
    error = "truncated PGM data: " + path.string();
    return std::nullopt;
  }

  // Value-to-occupancy lookup table (equivalent to map_server's trinary conversion)
  int8_t lut[256];
  for (int v = 0; v <= maxval; ++v) {
    const double occ = info.negate ?
      static_cast<double>(v) / maxval :
      static_cast<double>(maxval - v) / maxval;
    lut[v] = occ > info.occupied_thresh ? 100 : (occ < info.free_thresh ? 0 : -1);
  }

  // In PGM row 0 is the top, in OccupancyGrid row 0 is the bottom, so flip vertically while converting
  std::vector<int8_t> out(n);
  for (int row = 0; row < height; ++row) {
    const uint8_t * src = raw.data() + static_cast<std::size_t>(row) * width;
    int8_t * dst = out.data() + static_cast<std::size_t>(height - 1 - row) * width;
    for (int col = 0; col < width; ++col) {
      dst[col] = lut[src[col] > maxval ? maxval : src[col]];
    }
  }
  return out;
}

}  // namespace tile_map_server
