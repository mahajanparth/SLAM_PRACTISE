#include <cstddef>
#include <functional>
#include <list>
#include <unordered_map>
#include <utility>

template <typename Key, typename Value, typename Hash = std::hash<Key>>
class LRUCache {
 public:
  explicit LRUCache(std::size_t capacity) : cap_(capacity) {}

  Value* get(const Key& k) {
    // TODO
    return nullptr;
  }
  void put(const Key& k, Value v) {
    // TODO
  }
  bool contains(const Key& k) const {
    // TODO
    return false;
  }
  std::size_t size() const {
    // TODO
    return 0;
  }

 private:
  std::size_t cap_;
  // TODO: data structures
};
