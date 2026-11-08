#include <array>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <utility>

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;  // 72 bytes, lives wherever the object lives (stack here)

Mat3 identity() {
  Mat3 I{};  // {} zero-fills; `Mat3 I;` would be uninitialised
  for (std::size_t i = 0; i < 3; ++i) I[i][i] = 1.0;
  return I;
}

Mat3 transpose(const Mat3& M) {
  Mat3 T{};
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) T[j][i] = M[i][j];
  return T;
}

Mat3 matmul(const Mat3& A, const Mat3& B) {
  Mat3 C{};  // fresh output, so A/B aliasing each other (matmul(R, R)) is harmless
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t k = 0; k < 3; ++k)
      for (std::size_t j = 0; j < 3; ++j) C[i][j] += A[i][k] * B[k][j];
  return C;
}

Vec3 mulVec(const Mat3& M, const Vec3& v) {
  Vec3 r{};
  for (std::size_t i = 0; i < 3; ++i) r[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
  return r;
}

double det(const Mat3& M) {
  return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
         M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
         M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

std::pair<Vec3, double> normalize(const Vec3& v) {
  const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n == 0.0) return {Vec3{}, 0.0};
  return {Vec3{v[0] / n, v[1] / n, v[2] / n}, n};
}

std::tuple<double, std::size_t, std::size_t> maxAbsEntry(const Mat3& M) {
  std::size_t br = 0, bc = 0;
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      if (std::abs(M[i][j]) > std::abs(M[br][bc])) { br = i; bc = j; }  // strict >: first wins ties
  return {M[br][bc], br, bc};
}
