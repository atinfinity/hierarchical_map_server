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

/// タイルのLRUキャッシュ。get()はミス時にディスクからロードする。
/// ファイル欠損・読み込み失敗は nullptr をキャッシュして返す(=未知領域)。
/// スレッドセーフではない。ワーカースレッド1本からのみ使用すること。
class TileCache
{
public:
  using TileData = std::shared_ptr<const std::vector<int8_t>>;
  using ErrorCallback = std::function<void (const std::string &)>;

  TileCache(TilesetInfo info, std::size_t capacity, ErrorCallback on_error = nullptr);

  /// nullptr = タイル無し(未知領域として扱う)
  TileData get(const TileIndex & idx);

  std::size_t size() const {return entries_.size();}

private:
  TilesetInfo info_;
  std::size_t capacity_;
  ErrorCallback on_error_;

  // LRU: 先頭が最近使用
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
