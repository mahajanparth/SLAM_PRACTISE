#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

// One 3x3 * 3xN product (blocked + vectorized), then broadcast-add t to every column.
Eigen::Matrix3Xd transformCloud(const Eigen::Isometry3d& T, const Eigen::Matrix3Xd& P) {
  return (T.linear() * P).colwise() + T.translation();
}

std::vector<Eigen::Vector3d> transformCloud(const Eigen::Isometry3d& T, const std::vector<Eigen::Vector3d>& pts) {
  const Eigen::Matrix3d R = T.linear();
  const Eigen::Vector3d t = T.translation();
  std::vector<Eigen::Vector3d> out;
  out.reserve(pts.size());  // one allocation
  for (const auto& p : pts) out.emplace_back(R * p + t);
  return out;
}

// Zero-copy view: 3 rows (x,y,z), n columns, consecutive columns are 4 floats apart (skip intensity).
Eigen::Matrix3Xf transformRaw(const Eigen::Isometry3d& T, const float* xyzi, int n) {
  Eigen::Map<const Eigen::Matrix<float, 3, Eigen::Dynamic>, 0, Eigen::OuterStride<4>> P(xyzi, 3, n);
  const Eigen::Matrix3f R = T.linear().cast<float>();
  const Eigen::Vector3f t = T.translation().cast<float>();
  return (R * P).colwise() + t;
}
