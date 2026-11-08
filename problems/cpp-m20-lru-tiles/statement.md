# LRU Cache for Map Tiles

The localization module streams map tiles from disk as the robot drives. Keep the recently used ones in RAM under a
**byte budget**, and evict the **least recently used** tiles first.

```cpp
struct TileKey { int x, y, zoom; bool operator==(const TileKey& o) const; };   // given
struct TileKeyHash { std::size_t operator()(const TileKey& k) const noexcept; };
using Bytes = std::vector<std::uint8_t>;

class TileCache {
 public:
  explicit TileCache(std::size_t budget_bytes);
  const Bytes* get(const TileKey& k);          // nullptr on miss; on hit the tile becomes most-recently-used
  void put(const TileKey& k, Bytes data);      // insert or replace; becomes most-recently-used; then evict
  bool contains(const TileKey& k) const;       // does NOT change recency
  std::size_t size() const;                    // number of cached tiles
  std::size_t bytes() const;                   // sum of data.size() of cached tiles
  std::size_t hits() const;  std::size_t misses() const;  std::size_t evictions() const;
  std::vector<TileKey> keysMruToLru() const;   // for debugging/tests
};
```

**Rules**
- After every `put`, evict LRU tiles until `bytes() <= budget`. Each removed tile counts as one eviction.
- A tile with `data.size() > budget` is **not cached**. If that key was cached, the old entry is removed too (no eviction counted).
- Replacing a key changes its size accounting and does not count as an eviction.
- `get` counts a hit or a miss. `contains` counts neither.
- A pointer returned by `get` stays valid until the next `put`.
- `get`, `put`, `contains`: **O(1) average** (plus the evictions).

### Think about
- Why `std::list` + `unordered_map<Key, list::iterator>`? What does `splice` guarantee about iterators?
- Why is `x ^ y ^ zoom` a bad hash for tiles around the robot?
- How would you make it thread-safe? Can `get` take a shared (reader) lock?
- Returning `const Bytes*` vs `std::shared_ptr<const Bytes>`: what if the caller holds the tile across a `put`?
