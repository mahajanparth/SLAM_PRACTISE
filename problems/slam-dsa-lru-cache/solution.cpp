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
    auto f = index_.find(k);
    if (f == index_.end()) return nullptr;
    touch(f->second);
    return &f->second->second;  // list nodes never move, so this pointer is stable
  }

  void put(const Key& k, Value v) {
    if (cap_ == 0) return;
    auto f = index_.find(k);
    if (f != index_.end()) {  // overwrite in place, no eviction
      f->second->second = std::move(v);
      touch(f->second);
      return;
    }
    if (items_.size() == cap_) {  // evict LRU = back of the list
      index_.erase(items_.back().first);
      items_.pop_back();
    }
    items_.emplace_front(k, std::move(v));
    index_.emplace(k, items_.begin());
  }

  bool contains(const Key& k) const { return index_.count(k) > 0; }
  std::size_t size() const { return items_.size(); }

 private:
  using List = std::list<std::pair<Key, Value>>;  // front = most recently used
  // splice relinks the node in O(1); no copy, all iterators stay valid
  void touch(typename List::iterator it) { items_.splice(items_.begin(), items_, it); }

  std::size_t cap_;
  List items_;
  std::unordered_map<Key, typename List::iterator, Hash> index_;
};
