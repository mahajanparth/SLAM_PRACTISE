// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static int g_destroyed = 0;
struct Negate : Filter {  // a filter the candidate has never seen
  ~Negate() override { ++g_destroyed; }
  std::vector<double> apply(const std::vector<double>& in) const override {
    std::vector<double> o(in);
    for (double& v : o) v = -v;
    return o;
  }
  std::string name() const override { return "neg"; }
};

int main() {
  const std::vector<double> scan = {0.05, 2.0, 150.0, 4.0};

  ClampFilter clamp(0.1, 100.0);
  auto c = clamp.apply(scan);
  CHECK_EQ(c.size(), 4u);
  if (c.size() == 4) { CHECK_NEAR(c[0], 0.1, 1e-12); CHECK_NEAR(c[2], 100.0, 1e-12); CHECK_NEAR(c[1], 2.0, 1e-12); }

  MovingAverage ma(2);
  auto m = ma.apply({2.0, 4.0, 6.0, 8.0});
  CHECK_EQ(m.size(), 4u);
  if (m.size() == 4) { CHECK_NEAR(m[0], 2.0, 1e-12); CHECK_NEAR(m[1], 3.0, 1e-12); CHECK_NEAR(m[3], 7.0, 1e-12); }

  FilterChain chain;
  chain.add(std::make_unique<ClampFilter>(0.1, 100.0));
  chain.add(std::make_unique<ScaleFilter>(0.5));
  CHECK_EQ(chain.size(), 2u);
  CHECK_EQ(chain.describe(), std::string("clamp->scale"));
  auto out = chain.apply(scan);
  CHECK_EQ(out.size(), 4u);
  if (out.size() == 4) { CHECK_NEAR(out[2], 50.0, 1e-12); CHECK_NEAR(out[0], 0.05, 1e-12); }

  // Base-class pointer, unknown derived type
  std::unique_ptr<Filter> f = std::make_unique<ScaleFilter>(3.0);
  CHECK_EQ(f->name(), std::string("scale"));

#ifdef HIDDEN
  // empty chain is the identity
  FilterChain empty;
  CHECK_EQ(empty.describe(), std::string(""));
  CHECK(empty.apply(scan) == scan);
  // empty input
  CHECK(MovingAverage(3).apply({}).empty());
  CHECK(ClampFilter(0, 1).apply({}).empty());
  // window larger than input; window 1 = identity
  auto w = MovingAverage(10).apply({1.0, 2.0, 3.0});
  CHECK(w.size() == 3 && std::abs(w[2] - 2.0) < 1e-12);
  CHECK(MovingAverage(1).apply({5.0, -1.0, 7.0}) == (std::vector<double>{5.0, -1.0, 7.0}));
  // order matters: scale then clamp != clamp then scale
  FilterChain a;
  a.add(std::make_unique<ScaleFilter>(10.0));
  a.add(std::make_unique<ClampFilter>(0.0, 5.0));
  auto ao = a.apply({1.0});
  CHECK(ao.size() == 1 && std::abs(ao[0] - 5.0) < 1e-12);
  // nullptr ignored; user-defined filter dispatched and owned
  {
    FilterChain n;
    n.add(nullptr);
    n.add(std::make_unique<Negate>());
    n.add(std::make_unique<MovingAverage>(3));
    CHECK_EQ(n.size(), 2u);
    CHECK_EQ(n.describe(), std::string("neg->movavg"));
    auto no = n.apply({3.0, 6.0, 9.0, 12.0});
    CHECK(no.size() == 4 && std::abs(no[3] + 9.0) < 1e-12);
  }
  CHECK_EQ(g_destroyed, 1);  // chain destroyed its Negate through Filter*
  // large scan: O(n) moving average, values checked against brute force at a few indices
  std::vector<double> big(100000);
  for (std::size_t i = 0; i < big.size(); ++i) big[i] = static_cast<double>(i % 97);
  auto bo = MovingAverage(50).apply(big);
  bool ok = bo.size() == big.size();
  for (std::size_t i : {0ul, 10ul, 49ul, 50ul, 12345ul, 99999ul}) {
    if (!ok) break;
    double s = 0; std::size_t lo = i + 1 >= 50 ? i + 1 - 50 : 0;
    for (std::size_t j = lo; j <= i; ++j) s += big[j];
    ok = std::abs(bo[i] - s / static_cast<double>(i - lo + 1)) < 1e-6;
  }
  CHECK(ok);
#endif
  return TEST_SUMMARY();
}
