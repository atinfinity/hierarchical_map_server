#include "tile_map_server/tile_cache.hpp"

#include <utility>

#include "tile_map_server/pgm_loader.hpp"

namespace tile_map_server
{

TileCache::TileCache(TilesetInfo info, std::size_t capacity, ErrorCallback on_error)
: info_(std::move(info)), capacity_(capacity > 0 ? capacity : 1), on_error_(std::move(on_error))
{
}

TileCache::TileData TileCache::get(const TileIndex & idx)
{
  auto it = entries_.find(idx);
  if (it != entries_.end()) {
    lru_.splice(lru_.begin(), lru_, it->second.lru_it);
    return it->second.data;
  }

  TileData data;
  const auto path = info_.tilePath(idx);
  if (std::filesystem::exists(path)) {
    std::string error;
    auto occ = loadTileOccupancy(info_, path, error);
    if (occ) {
      data = std::make_shared<const std::vector<int8_t>>(std::move(*occ));
    } else if (on_error_) {
      on_error_(error);
    }
  }
  // ファイル欠損・破損も nullptr としてキャッシュし、毎回のディスクアクセスを避ける

  if (entries_.size() >= capacity_) {
    entries_.erase(lru_.back());
    lru_.pop_back();
  }
  lru_.push_front(idx);
  entries_.emplace(idx, Entry{data, lru_.begin()});
  return data;
}

}  // namespace tile_map_server
