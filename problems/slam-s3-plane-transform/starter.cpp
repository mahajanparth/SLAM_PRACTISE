#include <Eigen/Dense>
#include <Eigen/Geometry>

Eigen::Vector4d transformPlane(const Eigen::Isometry3d& T_B_A, const Eigen::Vector4d& plane_A) {
  // TODO
  return plane_A;
}

struct Line3 { Eigen::Vector3d point, dir; };

Line3 transformLine(const Eigen::Isometry3d& T_B_A, const Line3& line_A) {
  // TODO
  return line_A;
}
