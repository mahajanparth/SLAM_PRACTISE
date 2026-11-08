// The candidate's code is injected before this file (g++ -include).
#include <type_traits>
#include "common/check.h"

static double unitErr(const Rotation2D& r) { return std::abs(r.cos() * r.cos() + r.sin() * r.sin() - 1.0); }

int main() {
  const Rotation2D id;
  CHECK_NEAR(id.cos(), 1.0, 1e-12);
  const Rotation2D q(M_PI / 2);
  Vec2 v = q * Vec2{1, 0};
  CHECK_NEAR(v.x, 0.0, 1e-12);
  CHECK_NEAR(v.y, 1.0, 1e-12);

  const Rotation2D r = Rotation2D::fromCosSin(3, 4);
  CHECK_NEAR(r.cos(), 0.6, 1e-12);
  CHECK_NEAR(r.sin(), 0.8, 1e-12);

  CHECK_NEAR((Rotation2D(0.3) * Rotation2D(0.4)).angle(), 0.7, 1e-12);
  CHECK_NEAR(Rotation2D(0.5).inverse().angle(), -0.5, 1e-12);

#ifdef HIDDEN
  CHECK(!(std::is_convertible_v<double, Rotation2D>));
  // angle wrapping into (-pi, pi]
  CHECK_NEAR(Rotation2D(3 * M_PI / 2).angle(), -M_PI / 2, 1e-12);
  CHECK_NEAR(Rotation2D(M_PI).angle(), M_PI, 1e-12);
  CHECK_NEAR(Rotation2D(-M_PI).angle(), M_PI, 1e-12);
  CHECK_NEAR(Rotation2D(10.0).angle(), 10.0 - 4 * M_PI, 1e-12);
  // degenerate factories -> identity
  CHECK_NEAR(Rotation2D::fromCosSin(0, 0).cos(), 1.0, 1e-12);
  CHECK_NEAR(Rotation2D::fromVectors({0, 0}, {1, 1}).angle(), 0.0, 1e-12);
  // fromVectors: different lengths, negative quadrant
  CHECK_NEAR(Rotation2D::fromVectors({2, 0}, {0, -5}).angle(), -M_PI / 2, 1e-12);
  CHECK_NEAR(Rotation2D::fromVectors({1, 1}, {-3, -3}).angle(), M_PI, 1e-12);
  // composition order + inverse round trip
  Rotation2D a(1.0), b(-2.5);
  Vec2 p{0.3, -1.7};
  Vec2 ab = (a * b) * p, seq = a * (b * p);
  CHECK(std::abs(ab.x - seq.x) < 1e-12 && std::abs(ab.y - seq.y) < 1e-12);
  Vec2 back = (a.inverse() * a) * p;
  CHECK(std::abs(back.x - p.x) < 1e-12 && std::abs(back.y - p.y) < 1e-12);
  // invariant survives long chains and non-unit inputs
  Rotation2D acc;
  const Rotation2D step = Rotation2D::fromCosSin(1.0, 1e-3);
  for (int i = 0; i < 100000; ++i) acc = acc * step;
  CHECK(unitErr(acc) < 1e-9 && unitErr(Rotation2D::fromCosSin(-1e6, 2e6)) < 1e-12);
  CHECK_NEAR(acc.angle(), std::remainder(100000 * std::atan2(1e-3, 1.0), 2 * M_PI), 1e-6);
#endif
  return TEST_SUMMARY();
}
