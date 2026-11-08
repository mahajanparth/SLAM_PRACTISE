// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static bool sortedWithTies(const std::vector<StampedPoint>& v) {
  for (std::size_t i = 1; i < v.size(); ++i) {
    if (v[i].t < v[i - 1].t) return false;
    // x encodes the scan id, y the index within the scan (see makeScan)
    if (v[i].t == v[i - 1].t && (v[i].p.x < v[i - 1].p.x ||
                                 (v[i].p.x == v[i - 1].p.x && v[i].p.y < v[i - 1].p.y))) return false;
  }
  return true;
}
static std::vector<StampedPoint> makeScan(int id, std::vector<double> ts) {
  std::vector<StampedPoint> s;
  for (std::size_t i = 0; i < ts.size(); ++i) s.push_back({ts[i], {double(id), double(i), 0}});
  return s;
}

int main() {
  std::vector<Point3> dst = {{1, 1, 1}};
  std::vector<Point3> src = {{2, 2, 2}, {3, 3, 3}};
  appendScan(dst, src);
  CHECK(dst.size() == 3);
  if (dst.size() == 3) CHECK_NEAR(dst[2].x, 3.0, 1e-12);
  CHECK(src.size() == 2);

  std::vector<std::vector<StampedPoint>> scans = {makeScan(0, {0.0, 0.2, 0.4}), makeScan(1, {0.1, 0.2, 0.5}),
                                                  makeScan(2, {0.05})};
  auto m = mergeSorted(scans);
  CHECK(m.size() == 7);
  CHECK(sortedWithTies(m));
  CHECK(m.capacity() == m.size());  // exactly one allocation
  if (m.size() == 7) { CHECK_NEAR(m[1].t, 0.05, 1e-12); CHECK_NEAR(m[3].p.x, 0.0, 1e-12); }  // tie at 0.2: scan 0 first

#ifdef HIDDEN
  // self-append: the cloud doubles itself
  std::vector<Point3> self = {{1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  self.shrink_to_fit();
  appendScan(self, self);
  CHECK(self.size() == 6);
  if (self.size() == 6) { CHECK_NEAR(self[3].x, 1.0, 1e-12); CHECK_NEAR(self[5].x, 3.0, 1e-12); }
  // append empty / into empty
  std::vector<Point3> e;
  appendScan(e, src);
  CHECK(e.size() == 2);
  appendScan(e, std::vector<Point3>{});
  CHECK(e.size() == 2);
  // empty inputs
  CHECK(mergeSorted({}).empty());
  CHECK(mergeSorted({{}, {}, {}}).empty());
  // all identical timestamps: pure tie-break
  auto tie = mergeSorted({makeScan(0, {1, 1}), makeScan(1, {1}), makeScan(2, {1, 1})});
  CHECK(tie.size() == 5 && sortedWithTies(tie));
  // large: 8 interleaved scans
  std::vector<std::vector<StampedPoint>> big(8);
  for (int k = 0; k < 8; ++k) {
    std::vector<double> ts;
    for (int i = 0; i < 20000; ++i) ts.push_back(i * 8 + (7 - k) + (i % 5 == 0 ? 0.0 : 0.5));
    std::sort(ts.begin(), ts.end());
    big[k] = makeScan(k, ts);
  }
  auto bm = mergeSorted(big);
  CHECK(bm.size() == 160000);
  CHECK(sortedWithTies(bm));
  CHECK(bm.capacity() == bm.size());
#endif
  return TEST_SUMMARY();
}
