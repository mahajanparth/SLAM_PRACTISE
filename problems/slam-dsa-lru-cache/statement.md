# LRU Cache for Map Tiles

A localization node streams a large prior map from disk in tiles. Only `capacity` tiles fit in RAM; when
a new tile is loaded, evict the **least recently used** one.

```cpp
template <typename Key, typename Value, typename Hash = std::hash<Key>>
class LRUCache {
 public:
  explicit LRUCache(std::size_t capacity);
  Value* get(const Key& k);               // nullptr on a miss; a hit marks k as most recently used
  void put(const Key& k, Value v);        // insert or overwrite; marks k most recent; evicts LRU if over capacity
  bool contains(const Key& k) const;      // does NOT change recency
  std::size_t size() const;
};
```

### Requirements
- `get`, `put` and `contains` in **O(1)** average.
- `capacity == 0`: the cache stores nothing (`put` is a no-op).
- The pointer returned by `get` stays valid until that key is evicted (other operations must not move it).
- Overwriting an existing key does not evict anything.

### Think about
- Why `std::list` and not `std::vector` or `std::deque` for the recency order? What does `splice` cost?
- Iterator/pointer stability: which containers guarantee it?
- How would you make it thread-safe? Is a `shared_mutex` enough, given that `get` mutates recency?
- Tiles keyed by `(ix, iy)`: write the hash. Why is `ix ^ iy` a bad one?
