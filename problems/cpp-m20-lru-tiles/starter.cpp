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
    // TODO: combine x, y, zoom
    return 0;
  }
};

using Bytes = std::vector<std::uint8_t>;

class TileCache {
 public:
  explicit TileCache(std::size_t budget_bytes) : budget_(budget_bytes) {}

  const Bytes* get(const TileKey& k) {
    // TODO
    return nullptr;
  }
  void put(const TileKey& k, Bytes data) {
    // TODO
  }
  bool contains(const TileKey& k) const {
    // TODO
    return false;
  }
  std::size_t size() const { return 0; /* TODO */ }
  std::size_t bytes() const { return 0; /* TODO */ }
  std::size_t hits() const { return 0; /* TODO */ }
  std::size_t misses() const { return 0; /* TODO */ }
  std::size_t evictions() const { return 0; /* TODO */ }
  std::vector<TileKey> keysMruToLru() const {
    // TODO
    return {};
  }

 private:
  std::size_t budget_;
  // TODO: your data structures
};
