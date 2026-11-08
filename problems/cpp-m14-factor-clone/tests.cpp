// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

// A factor type the candidate has never seen: pulls x[key] towards 0 with weight w.
struct ZeroFactor : Factor {
  ZeroFactor(int k, double w) : k_(k), w_(w) {}
  double error(const std::vector<double>& x) const override { return w_ * x[k_]; }
  std::unique_ptr<Factor> clone() const override { return std::make_unique<ZeroFactor>(*this); }
  std::string type() const override { return "zero"; }
  int k_; double w_;
};

int main() {
  const std::vector<double> x = {0.0, 1.1, 2.0};

  PriorFactor p(0, 0.5, 0.5);
  CHECK_NEAR(p.error(x), -1.0, 1e-12);
  BetweenFactor b(1, 2, 1.0, 0.1);
  CHECK_NEAR(b.error(x), -1.0, 1e-9);

  FactorGraph g;
  g.add(std::make_unique<PriorFactor>(0, 0.5, 0.5));    // r = -1
  g.add(std::make_unique<BetweenFactor>(0, 1, 1.0, 0.1)); // r = 1
  g.add(std::make_unique<BetweenFactor>(1, 2, 1.0, 0.1)); // r = -1
  CHECK_NEAR(g.totalError(x), 1.5, 1e-9);
  auto counts = g.countByType();
  CHECK(counts == (std::map<std::string, int>{{"between", 2}, {"prior", 1}}));

  // clone keeps the dynamic type
  auto c = b.clone();
  CHECK(c != nullptr);
  if (c) { CHECK_EQ(c->type(), std::string("between")); CHECK_NEAR(c->error(x), -1.0, 1e-9); }

  // deep copy
  FactorGraph g2(g);
  g2.add(std::make_unique<PriorFactor>(2, 0.0, 1.0));
  CHECK_EQ(g.size(), 3u);
  CHECK_EQ(g2.size(), 4u);
  if (g2.size() == 4) CHECK(&g2.at(0) != &g.at(0));

  auto pf = parseFactor("between 0 1 1.5 0.2");
  CHECK(pf != nullptr);
  if (pf) { CHECK_EQ(pf->type(), std::string("between")); CHECK_NEAR(pf->error({0.0, 1.5}), 0.0, 1e-12); }

#ifdef HIDDEN
  // empty graph
  FactorGraph e;
  CHECK_NEAR(e.totalError({}), 0.0, 1e-15);
  CHECK(e.countByType().empty());
  // copy assignment is deep, replaces old contents, and copies an unknown subclass
  FactorGraph h;
  h.add(std::make_unique<ZeroFactor>(1, 2.0));
  h.add(std::make_unique<PriorFactor>(0, 0.0, 1.0));
  FactorGraph h2;
  h2.add(std::make_unique<PriorFactor>(0, 9.0, 1.0));
  h2 = h;
  CHECK_EQ(h2.size(), 2u);
  if (h2.size() == 2) {
    CHECK_EQ(h2.at(0).type(), std::string("zero"));
    CHECK(&h2.at(0) != &h.at(0));
  }
  CHECK_NEAR(h2.totalError({1.0, 1.0}), 0.5 * (4.0 + 1.0), 1e-12);
  // self-assignment
  FactorGraph& alias = h;
  h = alias;
  CHECK_EQ(h.size(), 2u);
  CHECK_NEAR(h.totalError({1.0, 1.0}), 2.5, 1e-12);
  // copy outlives the original
  FactorGraph* tmp = new FactorGraph(g);
  FactorGraph survivor(*tmp);
  delete tmp;
  CHECK_NEAR(survivor.totalError(x), 1.5, 1e-9);
  // moves still work (moved-from graph is valid)
  FactorGraph moved(std::move(survivor));
  CHECK_EQ(moved.size(), 3u);
  // factory edge cases
  auto pr = parseFactor("prior 3 -2.5 0.25");
  CHECK(pr != nullptr);
  if (pr) CHECK_NEAR(pr->error({0, 0, 0, -2.0}), 2.0, 1e-12);
  CHECK(parseFactor("") == nullptr);
  CHECK(parseFactor("odom 0 1 1.0 0.1") == nullptr);
  CHECK(parseFactor("prior 0 1.0") == nullptr);
  CHECK(parseFactor("prior 0 1.0 0.0") == nullptr);
  CHECK(parseFactor("prior 0 1.0 -0.1") == nullptr);
  CHECK(parseFactor("between -1 0 1.0 0.1") == nullptr);
  CHECK(parseFactor("between 0 1 abc 0.1") == nullptr);
  CHECK(parseFactor("between 0 1 1.0 0.1 extra") == nullptr);
  CHECK(parseFactor("  prior   1  2.0   1.0  ") != nullptr);
#endif
  return TEST_SUMMARY();
}
