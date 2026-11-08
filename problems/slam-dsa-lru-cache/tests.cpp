// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include <string>
#include <vector>

struct TileId { int ix, iy; bool operator==(const TileId& o) const { return ix == o.ix && iy == o.iy; } };
struct TileHash {
  std::size_t operator()(const TileId& t) const {
    return std::hash<long long>()((static_cast<long long>(t.ix) << 32) ^ static_cast<unsigned>(t.iy));
  }
};

int main() {
  LRUCache<int, std::string> c(2);
  c.put(1, "a");
  c.put(2, "b");
  CHECK(c.get(1) != nullptr);          // 1 is now most recent
  c.put(3, "c");                       // evicts 2
  CHECK(!c.contains(2));
  CHECK(c.contains(1));
  CHECK(c.contains(3));
  CHECK_EQ(c.size(), std::size_t(2));
  std::string* v = c.get(3);
  CHECK(v != nullptr && *v == "c");
  CHECK(c.get(42) == nullptr);

#ifdef HIDDEN
  // overwrite does not evict and updates recency
  LRUCache<int, int> o(2);
  o.put(1, 10); o.put(2, 20); o.put(1, 11);  // 1 most recent
  CHECK_EQ(o.size(), std::size_t(2));
  o.put(3, 30);                               // evicts 2
  CHECK(!o.contains(2));
  CHECK(o.get(1) && *o.get(1) == 11);
  // contains() must not refresh recency
  LRUCache<int, int> r(2);
  r.put(1, 1); r.put(2, 2);
  (void)r.contains(1);
  r.put(3, 3);                                 // still evicts 1
  CHECK(!r.contains(1));
  // capacity 0 and 1
  LRUCache<int, int> z(0);
  z.put(1, 1);
  CHECK_EQ(z.size(), std::size_t(0));
  CHECK(z.get(1) == nullptr);
  LRUCache<int, int> one(1);
  one.put(1, 1); one.put(2, 2);
  CHECK(!one.contains(1) && one.contains(2));
  // pointer stability + custom hash key
  LRUCache<TileId, std::vector<float>, TileHash> tiles(100);
  tiles.put({0, 0}, std::vector<float>(10, 1.f));
  std::vector<float>* p0 = tiles.get({0, 0});
  for (int i = 1; i < 99; ++i) tiles.put({i, -i}, {});
  CHECK(p0 == tiles.get({0, 0}));
  // large: O(1) behaviour, final content check
  LRUCache<int, int> big(1000);
  for (int i = 0; i < 200000; ++i) { big.put(i, i); if (i % 3 == 0) big.get(i / 2); }
  CHECK_EQ(big.size(), std::size_t(1000));
  CHECK(big.contains(199999) && !big.contains(0));
#endif
  return TEST_SUMMARY();
}
