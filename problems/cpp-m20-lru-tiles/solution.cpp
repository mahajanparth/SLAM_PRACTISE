#include <cstddef>
#include <cstdint>
#include <iterator>
#include <list>
#include <unordered_map>
#include <utility>
#include <vector>

struct TileKey {
  int x, y, zoom;
  bool operator==(const TileKey& o) const { return x == o.x && y == o.y && zoom == o.zoom; }
};

struct TileKeyHash {
  std::size_t operator()(const TileKey& k) const noexcept {
    // boost::hash_combine-style mixing: order-sensitive, spreads small neighbouring coords.
    std::size_t h = std::hash<int>{}(k.x);
    auto mix = [&h](int v) { h ^= std::hash<int>{}(v) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2); };
    mix(k.y);
    mix(k.zoom);
    return h;
  }
};

using Bytes = std::vector<std::uint8_t>;

class TileCache {
 public:
  explicit TileCache(std::size_t budget_bytes) : budget_(budget_bytes) {}

  const Bytes* get(const TileKey& k) {
    auto it = index_.find(k);
    if (it == index_.end()) { ++misses_; return nullptr; }
    ++hits_;
    lru_.splice(lru_.begin(), lru_, it->second);  // O(1), node (and its data) doesn't move in memory
    return &it->second->data;
  }

  void put(const TileKey& k, Bytes data) {
    auto it = index_.find(k);
    if (data.size() > budget_) {  // can never fit: drop any stale copy and don't cache
      if (it != index_.end()) erase(it);
      return;
    }
    if (it != index_.end()) {
      bytes_ = bytes_ - it->second->data.size() + data.size();
      it->second->data = std::move(data);
      lru_.splice(lru_.begin(), lru_, it->second);
    } else {
      bytes_ += data.size();
      lru_.push_front(Entry{k, std::move(data)});
      index_.emplace(k, lru_.begin());
    }
    while (bytes_ > budget_) {  // evict from the LRU end; the new tile is at the front and fits on its own
      erase(index_.find(std::prev(lru_.end())->key));
      ++evictions_;
    }
  }

  bool contains(const TileKey& k) const { return index_.count(k) != 0; }
  std::size_t size() const { return index_.size(); }
  std::size_t bytes() const { return bytes_; }
  std::size_t hits() const { return hits_; }
  std::size_t misses() const { return misses_; }
  std::size_t evictions() const { return evictions_; }
  std::vector<TileKey> keysMruToLru() const {
    std::vector<TileKey> out;
    out.reserve(lru_.size());
    for (const auto& e : lru_) out.push_back(e.key);
    return out;
  }

 private:
  struct Entry { TileKey key; Bytes data; };  // key stored in the node so eviction can find the map entry
  using List = std::list<Entry>;
  using Index = std::unordered_map<TileKey, List::iterator, TileKeyHash>;

  void erase(Index::iterator it) {
    bytes_ -= it->second->data.size();
    lru_.erase(it->second);
    index_.erase(it);
  }

  std::size_t budget_;
  std::size_t bytes_ = 0, hits_ = 0, misses_ = 0, evictions_ = 0;
  List lru_;     // front = most recently used
  Index index_;  // key -> node in lru_
};
