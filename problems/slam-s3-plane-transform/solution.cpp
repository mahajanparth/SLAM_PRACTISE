#include <Eigen/Dense>
#include <Eigen/Geometry>

// x_A = R^T (x_B - t)  =>  n_A^T R^T x_B - n_A^T R^T t + d_A = 0
//                       =>  n_B = R n_A,  d_B = d_A - n_B^T t      (pi_B = T^{-T} pi_A)
Eigen::Vector4d transformPlane(const Eigen::Isometry3d& T_B_A, const Eigen::Vector4d& plane_A) {
  const Eigen::Vector3d n_B = T_B_A.linear() * plane_A.head<3>();
  const double d_B = plane_A(3) - n_B.dot(T_B_A.translation());
  return Eigen::Vector4d(n_B.x(), n_B.y(), n_B.z(), d_B);
}

struct Line3 { Eigen::Vector3d point, dir; };

// A point gets the full transform, a direction only the rotation (homogeneous w = 0).
Line3 transformLine(const Eigen::Isometry3d& T_B_A, const Line3& line_A) {
  return {T_B_A * line_A.point, T_B_A.linear() * line_A.dir};
}
