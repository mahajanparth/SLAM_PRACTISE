// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"
#include "common/cloud_utils.h"

// Point at the CENTRE of pixel (row, col) for the default sensor model, at the given range.
static Eigen::Vector3d atPixel(int row, int col, double range, int rows = 32, int cols = 1800,
                               double fd = -30.0, double fu = 10.0) {
  const double up = fu * M_PI / 180, down = fd * M_PI / 180;
  const double el = up - (row + 0.5) * (up - down) / rows;
  const double az = -M_PI + (col + 0.5) * 2 * M_PI / cols;
  return range * Eigen::Vector3d(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
}

int main() {
  cu::Cloud pts = {atPixel(0, 0, 5.0), atPixel(10, 900, 12.0), atPixel(31, 1799, 3.0), atPixel(10, 900, 7.0)};
  RangeImage img = projectToRangeImage(pts);
  CHECK(img.rows == 32 && img.cols == 1800);
  CHECK(img.range.size() == 32u * 1800u && img.index.size() == 32u * 1800u);
  if (img.range.size() == 32u * 1800u && img.index.size() == 32u * 1800u) {
    CHECK_NEAR(img.at(0, 0), 5.0, 1e-9);
    CHECK_NEAR(img.at(31, 1799), 3.0, 1e-9);
    CHECK_NEAR(img.at(10, 900), 7.0, 1e-9);  // nearest of the two kept
    CHECK_EQ(img.index[10 * 1800 + 900], 3);
    int filled = 0;
    for (double r : img.range) filled += r >= 0;
    CHECK_EQ(filled, 3);
  }
  // a point straight ahead (+x, el = 0): az = 0 -> col 900; el = 0 -> row floor(10/40*32) = 8
  RangeImage f = projectToRangeImage({Eigen::Vector3d(10, 0, 0)});
  if (f.index.size() == 32u * 1800u) CHECK_EQ(f.index[8 * 1800 + 900], 0);
  else CHECK(false);

#ifdef HIDDEN
  // out of FOV (above +10 deg, below -30 deg) and the origin are discarded
  RangeImage o = projectToRangeImage({Eigen::Vector3d(1, 0, 0.5), Eigen::Vector3d(1, 0, -1.0), Eigen::Vector3d(0, 0, 0)});
  bool empty = o.index.size() == 32u * 1800u;
  for (int v : o.index) empty &= v == -1;
  CHECK(empty);
  // empty cloud -> full-size empty image
  RangeImage e = projectToRangeImage({});
  CHECK(e.range.size() == 32u * 1800u);
  if (!e.range.empty()) CHECK_NEAR(e.range[0], -1.0, 0.0);
  // directly behind the sensor (az = +pi) -> last column (clamped)
  RangeImage b = projectToRangeImage({Eigen::Vector3d(-5, 0, 0)});
  if (b.index.size() == 32u * 1800u) CHECK_EQ(b.index[8 * 1800 + 1799], 0);
  else CHECK(false);
  // just left of behind (y slightly negative) -> column 0
  RangeImage b2 = projectToRangeImage({Eigen::Vector3d(-5, -1e-6, 0)});
  if (b2.index.size() == 32u * 1800u) CHECK_EQ(b2.index[8 * 1800 + 0], 0);
  else CHECK(false);
  // equal ranges: keep the earlier point
  RangeImage t = projectToRangeImage({atPixel(5, 5, 2.0), atPixel(5, 5, 2.0)});
  if (t.index.size() == 32u * 1800u) CHECK_EQ(t.index[5 * 1800 + 5], 0);
  else CHECK(false);
  // a custom sensor (16 x 360, -15..15 deg) and many points at random pixel centres
  bool ok = true;
  std::uniform_int_distribution<int> ur(0, 15), uc(0, 359);
  std::uniform_real_distribution<double> urange(1.0, 80.0);
  std::vector<int> want(16 * 360, -1);
  std::vector<double> wr(16 * 360, 1e9);
  cu::Cloud many;
  for (int i = 0; i < 3000; ++i) {
    int r = ur(cu::rng()), c = uc(cu::rng());
    double rg = urange(cu::rng());
    many.push_back(atPixel(r, c, rg, 16, 360, -15, 15));
    if (rg < wr[r * 360 + c]) { wr[r * 360 + c] = rg; want[r * 360 + c] = i; }
  }
  RangeImage m = projectToRangeImage(many, 16, 360, -15, 15);
  ok = m.index.size() == want.size() && m.index == want;
  CHECK(ok);
  CHECK(m.rows == 16 && m.cols == 360);
#endif
  return TEST_SUMMARY();
}
