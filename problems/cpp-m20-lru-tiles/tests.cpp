// The candidate's code is injected before this file (g++ -include).
#include <algorithm>
#include <random>
#include <unordered_set>
#include "common/check.h"

static Bytes tile(std::size_t n, std::uint8_t v) { return Bytes(n, v); }
static bool keysEq(const std::vector<TileKey>& a, const std::vector<TileKey>& b) { return a == b; }

int main() {
  TileCache c(300);
  c.put({0, 0, 1}, tile(100, 1));
  c.put({1, 0, 1}, tile(100, 2));
  c.put({2, 0, 1}, tile(100, 3));
  CHECK_EQ(c.size(), 3u);
  CHECK_EQ(c.bytes(), 300u);

  const Bytes* t = c.get({0, 0, 1});  // (0,0) becomes MRU
  CHECK(t != nullptr && t->size() == 100 && (*t)[0] == 1);
  CHECK(c.get({9, 9, 1}) == nullptr);

  c.put({3, 0, 1}, tile(100, 4));     // evicts LRU = (1,0)
  CHECK(!c.contains({1, 0, 1}));
  CHECK(c.contains({0, 0, 1}));
  CHECK_EQ(c.evictions(), 1u);
  CHECK(c.hits() == 1 && c.misses() == 1);
  CHECK(keysEq(c.keysMruToLru(), {{3, 0, 1}, {0, 0, 1}, {2, 0, 1}}));

#ifdef HIDDEN
  // contains() must not change recency
  CHECK(c.contains({2, 0, 1}));
  c.put({4, 0, 1}, tile(100, 5));     // still evicts (2,0)
  CHECK(!c.contains({2, 0, 1}));
  // replace grows a tile -> evicts others, not itself; no eviction counted for the replace
  TileCache r(250);
  r.put({0, 0, 0}, tile(100, 1));
  r.put({1, 0, 0}, tile(100, 2));
  r.put({0, 0, 0}, tile(200, 9));     // now 300 bytes -> evict (1,0)
  CHECK(r.size() == 1 && r.bytes() == 200 && r.evictions() == 1);
  const Bytes* g = r.get({0, 0, 0});
  CHECK(g && g->size() == 200 && (*g)[0] == 9);
  // big new tile evicts several
  TileCache m(300);
  for (int i = 0; i < 3; ++i) m.put({i, 0, 0}, tile(100, 1));
  m.put({7, 7, 0}, tile(250, 1));
  CHECK(m.size() == 1 && m.evictions() == 3 && m.bytes() == 250);
  // oversized tile: not cached, removes stale copy
  m.put({7, 7, 0}, tile(301, 2));
  CHECK(m.size() == 0 && m.bytes() == 0 && !m.contains({7, 7, 0}));
  // zero budget and zero-size tiles
  TileCache z(0);
  z.put({1, 1, 1}, tile(1, 1));
  CHECK(z.size() == 0);
  z.put({1, 1, 1}, Bytes{});
  CHECK(z.size() == 1 && z.get({1, 1, 1}) != nullptr);
  // distinct keys that a weak hash would confuse
  TileCache k(1000);
  k.put({1, 2, 3}, tile(1, 1));
  k.put({2, 1, 3}, tile(1, 2));
  k.put({3, 2, 1}, tile(1, 3));
  CHECK(k.size() == 3 && (*k.get({2, 1, 3}))[0] == 2);
  TileKeyHash h;
  std::unordered_set<std::size_t> hs;
  for (int x = -20; x < 20; ++x) for (int y = -20; y < 20; ++y) hs.insert(h({x, y, 18}));
  CHECK(hs.size() >= 1550);
  // randomized against a simple reference (vector-based O(n) LRU)
  std::mt19937 rng(11);
  std::uniform_int_distribution<int> key(0, 30), sz(1, 60), op(0, 2);
  TileCache rc(500);
  std::vector<std::pair<TileKey, std::size_t>> ref;  // front = MRU
  bool ok = true;
  for (int step = 0; step < 4000 && ok; ++step) {
    TileKey kk{key(rng), 0, 5};
    auto pos = std::find_if(ref.begin(), ref.end(), [&](const auto& e) { return e.first == kk; });
    if (op(rng) == 0) {
      const Bytes* got = rc.get(kk);
      ok = (got != nullptr) == (pos != ref.end());
      if (pos != ref.end()) { auto e = *pos; ref.erase(pos); ref.insert(ref.begin(), e); ok = ok && got->size() == e.second; }
    } else {
      std::size_t s = sz(rng);
      if (pos != ref.end()) ref.erase(pos);
      ref.insert(ref.begin(), {kk, s});
      rc.put(kk, tile(s, 0));
      std::size_t tot = 0;
      for (auto& e : ref) tot += e.second;
      while (tot > 500) { tot -= ref.back().second; ref.pop_back(); }
    }
    ok = ok && rc.size() == ref.size();
  }
  std::vector<TileKey> refKeys;
  for (auto& e : ref) refKeys.push_back(e.first);
  CHECK(ok);
  CHECK(keysEq(rc.keysMruToLru(), refKeys));
#endif
  return TEST_SUMMARY();
}
