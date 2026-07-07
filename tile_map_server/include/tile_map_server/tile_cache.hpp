#ifndef TILE_MAP_SERVER__TILE_CACHE_HPP_
#define TILE_MAP_SERVER__TILE_CACHE_HPP_

#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "tile_map_server/tileset.hpp"

namespace tile_map_server
{

/// LRU cache of tiles. get() loads from disk on a miss.
/// A missing file or failed read caches and returns nullptr (= unknown region).
/// Not thread-safe. Use only from a single worker thread.
class TileCache
{
public:
  using TileData = std::shared_ptr<const std::vector<int8_t>>;
  using ErrorCallback = std::function<void (const std::string &)>;

  TileCache(TilesetInfo info, std::size_t capacity, ErrorCallback on_error = nullptr);

  /// nullptr = no tile (treated as unknown region)
  TileData get(const TileIndex & idx);

  std::size_t size() const {return entries_.size();}

private:
  TilesetInfo info_;
  std::size_t capacity_;
  ErrorCallback on_error_;

  // LRU: front is most recently used
  std::list<TileIndex> lru_;
  struct Entry
  {
    TileData data;
    std::list<TileIndex>::iterator lru_it;
  };
  std::unordered_map<TileIndex, Entry, TileIndexHash> entries_;
};

}  // namespace tile_map_server

#endif  // TILE_MAP_SERVER__TILE_CACHE_HPP_
