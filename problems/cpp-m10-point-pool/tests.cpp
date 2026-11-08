// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  CHECK_EQ(alignUp(13, 8), std::uintptr_t(16));
  CHECK_EQ(alignUp(64, 32), std::uintptr_t(64));

  PointPool pool(4);
  std::vector<Point3*> ptrs;
  for (int i = 0; i < 10; ++i) ptrs.push_back(pool.allocate(i, 2.0 * i, -i));
  CHECK_EQ(pool.size(), std::size_t(10));
  CHECK_EQ(pool.blockCount(), std::size_t(3));
  CHECK_EQ(pool.capacity(), std::size_t(12));
  bool stable = true;
  for (int i = 0; i < 10; ++i) stable = stable && ptrs[i] && ptrs[i]->x == i && ptrs[i]->y == 2.0 * i;
  CHECK(stable);

  pool.deallocate(ptrs[3]);
  Point3* again = pool.allocate(7, 7, 7);
  CHECK(again == ptrs[3] && again != nullptr);
  CHECK(pool.size() == 10 && pool.capacity() == 12);

#ifdef HIDDEN
  CHECK_EQ(alignUp(0, 64), std::uintptr_t(0));
  CHECK_EQ(alignUp(65, 64), std::uintptr_t(128));
  CHECK_EQ(alignUp(5, 1), std::uintptr_t(5));
  // lazy allocation, block_size 0 -> 1
  PointPool lazy(0);
  CHECK(lazy.blockCount() == 0 && lazy.capacity() == 0 && lazy.size() == 0);
  Point3* a = lazy.allocate(1, 2, 3);
  Point3* b = lazy.allocate(4, 5, 6);
  CHECK(lazy.blockCount() == 2 && a && b && a != b);
  // LIFO reuse; nullptr no-op
  lazy.deallocate(a);
  lazy.deallocate(b);
  lazy.deallocate(nullptr);
  CHECK_EQ(lazy.size(), std::size_t(0));
  Point3* c = lazy.allocate(0, 0, 0);
  Point3* d = lazy.allocate(0, 0, 1);
  CHECK(c == b && d == a && lazy.blockCount() == 2 && d && d->z == 1);
  // big: stability, alignment, block count
  PointPool big(1024);
  std::vector<Point3*> many;
  many.reserve(100000);
  for (int i = 0; i < 100000; ++i) many.push_back(big.allocate(i, 0, 0));
  CHECK_EQ(big.blockCount(), std::size_t(98));
  bool ok = true, aligned = true;
  for (int i = 0; i < 100000; ++i) {
    ok = ok && many[i] && many[i]->x == i;
    aligned = aligned && reinterpret_cast<std::uintptr_t>(many[i]) % alignof(Point3) == 0;
  }
  CHECK(ok && aligned);
  // free half, reallocate half: no new blocks
  for (int i = 0; i < 100000; i += 2) big.deallocate(many[i]);
  CHECK_EQ(big.size(), std::size_t(50000));
  for (int i = 0; i < 50000; ++i) big.allocate(-1, -1, -1);
  CHECK(big.blockCount() == 98 && big.size() == 100000 && many[1] && many[1]->x == 1);
#endif
  return TEST_SUMMARY();
}
