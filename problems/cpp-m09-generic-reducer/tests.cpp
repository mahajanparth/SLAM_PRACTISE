// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

int main() {
  auto range = [](const LidarPoint& p) { return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); };
  auto isFar = compose(range, [](double r) { return r > 30.0; });
  CHECK(isFar(LidarPoint{40, 0, 0, 1.f, 0}));
  CHECK(!isFar(LidarPoint{3, 4, 0, 1.f, 0}));

  auto inBand = allOf([](double v) { return v > 0; }, [](double v) { return v < 10; });
  CHECK(inBand(5.0));
  CHECK(!inBand(12.0));

  std::vector<LidarPoint> scan = {{1, 0, 0, 10.f, 0}, {2, 0, 0, 20.f, 1}, {3, 0, 0, 30.f, 0},
                                  {4, 0, 0, 40.f, 1}, {5, 0, 0, 50.f, 1}};
  auto counts = groupReduce(scan, [](const auto& p) { return p.ring; }, 0,
                            [](int c, const LidarPoint&) { return c + 1; });
  CHECK(counts.size() == 2 && counts[0] == 2 && counts[1] == 3);

  auto mean = makeRunningMean();
  mean(2.0);
  CHECK_NEAR(mean(4.0), 3.0, 1e-12);

#ifdef HIDDEN
  // compose works with any argument type and chains
  auto twice = compose([](auto v) { return v * 2; }, [](auto v) { return v + 1; });
  CHECK_EQ(twice(5), 11);
  CHECK_NEAR(twice(0.25), 1.5, 1e-12);
  auto toBin = compose(range, [](double r) { return static_cast<int>(r / 5.0); });  // 5 m range bins
  CHECK_EQ(toBin(LidarPoint{0, 0, -12, 0.f, 0}), 2);
  // compose owns its callables (built from temporaries in an inner scope)
  auto makeOffset = [] { double off = 100; return compose([off](double v) { return v + off; }, [](double v) { return -v; }); };
  auto neg = makeOffset();
  CHECK_NEAR(neg(1.0), -101.0, 1e-12);
  // allOf: empty pack, short-circuit, mixed predicate types
  CHECK(allOf()(42));
  int evaluated = 0;
  auto sc = allOf([&](int) { ++evaluated; return false; }, [&](int) { ++evaluated; return true; });
  CHECK(!sc(1));
  CHECK_EQ(evaluated, 1);
  // groupReduce: empty input, string keys, max-range per ring, mean intensity via pair accumulator
  CHECK(groupReduce(std::vector<LidarPoint>{}, [](const LidarPoint& p) { return p.ring; }, 0.0,
                    [](double a, const LidarPoint&) { return a; }).empty());
  auto parity = groupReduce(scan, [](const LidarPoint& p) { return p.ring % 2 ? std::string("odd") : std::string("even"); },
                            0.0, [](double s, const LidarPoint& p) { return s + p.intensity; });
  CHECK(parity.size() == 2 && std::abs(parity["even"] - 40.0) < 1e-9 && std::abs(parity["odd"] - 110.0) < 1e-9);
  auto maxR = groupReduce(scan, [](const LidarPoint& p) { return p.ring; }, 0.0,
                          [&](double m, const LidarPoint& p) { return std::max(m, range(p)); });
  CHECK(maxR.size() == 2 && maxR[0] == 3.0 && maxR[1] == 5.0);
  // running mean: independent accumulators, many samples, negative values
  auto m1 = makeRunningMean();
  auto m2 = makeRunningMean();
  double last = 0;
  for (int i = 1; i <= 100000; ++i) last = m1(double(i));
  CHECK_NEAR(last, 50000.5, 1e-6);
  CHECK_NEAR(m2(-7.0), -7.0, 1e-12);
#endif
  return TEST_SUMMARY();
}
