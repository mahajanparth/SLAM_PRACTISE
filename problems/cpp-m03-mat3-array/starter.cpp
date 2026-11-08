#include <array>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <utility>

using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;

Mat3 identity() {
  // TODO
  return Mat3{};
}

Mat3 transpose(const Mat3& M) {
  // TODO
  return Mat3{};
}

Mat3 matmul(const Mat3& A, const Mat3& B) {
  // TODO
  return Mat3{};
}

Vec3 mulVec(const Mat3& M, const Vec3& v) {
  // TODO
  return Vec3{};
}

double det(const Mat3& M) {
  // TODO
  return 0.0;
}

std::pair<Vec3, double> normalize(const Vec3& v) {
  // TODO
  return {Vec3{}, -1.0};
}

std::tuple<double, std::size_t, std::size_t> maxAbsEntry(const Mat3& M) {
  // TODO
  return {0.0, 9, 9};
}
