// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static double halve(double v) { return v / 2; }
static double addOne(double v) { return v + 1; }

int main() {
  CHECK_NEAR(applyN(halve, 16.0, 3), 2.0, 1e-12);
  CHECK_NEAR(applyN(addOne, 0.0, 5), 5.0, 1e-12);

  std::vector<Point3> c = {{1, 2, 3}, {4, 5, -6}, {0, 0, 10}};
  CHECK_NEAR(sumField(c, &Point3::y), 7.0, 1e-12);
  CHECK_NEAR(sumField(c, &Point3::z), 7.0, 1e-12);

  auto f = makeFilter("max_z", 5.0);
  CHECK(static_cast<bool>(f));
  CHECK(!makeFilter("no_such_filter", 1.0));

  std::vector<PointPredicate> stages = {makeFilter("min_z", -1.0), makeFilter("max_range", 9.0)};
  auto kept = applyPipeline(c, stages);
  CHECK(kept.size() == 1 && kept[0].x == 1);

#ifdef HIDDEN
  CHECK_NEAR(applyN(nullptr, 3.5, 4), 3.5, 1e-12);
  CHECK_NEAR(applyN(halve, 3.5, 0), 3.5, 1e-12);
  CHECK_NEAR(applyN([](double v) { return v * v; }, 2.0, 3), 256.0, 1e-12);  // capture-less lambda -> fn ptr
  CHECK_NEAR(sumField({}, &Point3::x), 0.0, 1e-12);
  // filter semantics (inclusive bounds)
  auto mr = makeFilter("max_range", 5.0);
  auto mx = makeFilter("min_x", 0.0);
  CHECK(mr && mx && mr({3, 4, 0}) && !mr({3, 4, 0.1}) && mx({0, -5, 0}) && !mx({-0.01, 0, 0}));
  CHECK(!makeFilter("MAX_Z", 1.0));  // case-sensitive
  // empty pipeline keeps everything; empty stages are skipped
  CHECK_EQ(applyPipeline(c, {}).size(), std::size_t(3));
  std::vector<PointPredicate> withHole = {PointPredicate{}, makeFilter("max_z", 3.0), makeFilter("bogus", 0)};
  auto k2 = applyPipeline(c, withHole);
  CHECK(k2.size() == 2 && k2[0].z == 3 && k2[1].z == -6);
  // order preserved on a bigger cloud
  std::vector<Point3> big;
  for (int i = 0; i < 50000; ++i) big.push_back({double(i % 100) - 50, 0, double(i % 7) - 3});
  auto kb = applyPipeline(big, {makeFilter("min_x", 0), makeFilter("min_z", 0)});
  std::vector<Point3> expect;
  for (const auto& p : big) if (p.x >= 0 && p.z >= 0) expect.push_back(p);
  bool same = kb.size() == expect.size();
  for (std::size_t i = 0; same && i < kb.size(); ++i) same = kb[i].x == expect[i].x && kb[i].z == expect[i].z;
  CHECK(!kb.empty() && same);
#endif
  return TEST_SUMMARY();
}
