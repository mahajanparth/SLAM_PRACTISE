// The candidate's code is injected before this file (g++ -include).
#include "common/check.h"

static Mat3 rotZ(double a) {
  return Mat3{Vec3{std::cos(a), -std::sin(a), 0}, Vec3{std::sin(a), std::cos(a), 0}, Vec3{0, 0, 1}};
}
static double maxDiff(const Mat3& A, const Mat3& B) {
  double m = 0;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) m = std::max(m, std::abs(A[i][j] - B[i][j]));
  return m;
}

int main() {
  const double PI = std::acos(-1.0);
  CHECK(identity() == (Mat3{Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}));  // std::array has ==
  Mat3 R = rotZ(PI / 2);
  Vec3 y = mulVec(R, Vec3{1, 0, 0});
  CHECK_NEAR(y[1], 1.0, 1e-12);
  CHECK_NEAR(maxDiff(matmul(R, transpose(R)), identity()), 0.0, 1e-12);  // R R^T = I
  CHECK_NEAR(det(R), 1.0, 1e-12);

  auto [u, n] = normalize(Vec3{3, 0, 4});
  CHECK_NEAR(n, 5.0, 1e-12);
  CHECK_NEAR(u[2], 0.8, 1e-12);

  auto [val, r, c] = maxAbsEntry(Mat3{Vec3{1, 2, 3}, Vec3{-7, 5, 0}, Vec3{6, 0, 1}});
  CHECK_NEAR(val, -7.0, 1e-12);
  CHECK(r == 1 && c == 0);

#ifdef HIDDEN
  // aliasing: R*R = rotation by pi
  CHECK_NEAR(maxDiff(matmul(R, R), rotZ(PI)), 0.0, 1e-12);
  // non-symmetric product order matters
  Mat3 A{Vec3{1, 2, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}, B{Vec3{1, 0, 0}, Vec3{3, 1, 0}, Vec3{0, 0, 2}};
  Mat3 AB = matmul(A, B);
  CHECK_NEAR(AB[0][0], 7.0, 1e-12);
  CHECK_NEAR(AB[2][2], 2.0, 1e-12);
  CHECK_NEAR(det(AB), det(A) * det(B), 1e-12);
  // singular and reflection
  CHECK_NEAR(det(Mat3{Vec3{1, 2, 3}, Vec3{2, 4, 6}, Vec3{0, 1, 1}}), 0.0, 1e-12);
  CHECK_NEAR(det(Mat3{Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, -1}}), -1.0, 1e-12);
  // transpose of non-symmetric
  CHECK_NEAR(transpose(A)[1][0], 2.0, 1e-12);
  // zero vector, negative vector
  auto [z, zn] = normalize(Vec3{0, 0, 0});
  CHECK(zn == 0.0 && z == (Vec3{0, 0, 0}));
  auto [m, mn] = normalize(Vec3{0, -2, 0});
  CHECK_NEAR(mn, 2.0, 1e-12);
  CHECK_NEAR(m[1], -1.0, 1e-12);
  // ties -> first in row-major order; all zeros -> (0,0,0)
  auto [tv, tr, tc] = maxAbsEntry(Mat3{Vec3{0, 4, 0}, Vec3{-4, 0, 0}, Vec3{0, 0, 4}});
  CHECK(tv == 4.0 && tr == 0 && tc == 1);
  auto [zv, zr, zc] = maxAbsEntry(Mat3{});
  CHECK(zv == 0.0 && zr == 0 && zc == 0);
#endif
  return TEST_SUMMARY();
}
