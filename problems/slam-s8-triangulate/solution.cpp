#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

Eigen::Vector3d triangulateDLT(const Eigen::Isometry3d& T1_cw, const Eigen::Vector2d& x1,
                               const Eigen::Isometry3d& T2_cw, const Eigen::Vector2d& x2) {
  // P = [R | t]; [x;1] x (P X) = 0 gives two independent rows per view.
  const Eigen::Matrix<double, 3, 4> P1 = T1_cw.matrix().topRows<3>();
  const Eigen::Matrix<double, 3, 4> P2 = T2_cw.matrix().topRows<3>();
  Eigen::Matrix4d A;
  A.row(0) = x1.x() * P1.row(2) - P1.row(0);
  A.row(1) = x1.y() * P1.row(2) - P1.row(1);
  A.row(2) = x2.x() * P2.row(2) - P2.row(0);
  A.row(3) = x2.y() * P2.row(2) - P2.row(1);
  // Null vector = right singular vector of the smallest singular value.
  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d Xh = svd.matrixV().col(3);
  return Xh.head<3>() / Xh(3);
}

Eigen::Vector3d triangulateMidpoint(const Eigen::Isometry3d& T1_cw, const Eigen::Vector2d& x1,
                                    const Eigen::Isometry3d& T2_cw, const Eigen::Vector2d& x2) {
  const Eigen::Matrix3d R1t = T1_cw.linear().transpose(), R2t = T2_cw.linear().transpose();
  const Eigen::Vector3d C1 = -R1t * T1_cw.translation(), C2 = -R2t * T2_cw.translation();
  const Eigen::Vector3d d1 = R1t * x1.homogeneous(), d2 = R2t * x2.homogeneous();
  // min_{s,u} |C1 + s d1 - C2 - u d2|^2  ->  normal equations (2x2)
  Eigen::Matrix2d M;
  M << d1.dot(d1), -d1.dot(d2),
      -d1.dot(d2),  d2.dot(d2);
  const Eigen::Vector3d w = C2 - C1;
  const Eigen::Vector2d rhs(d1.dot(w), -d2.dot(w));
  const Eigen::Vector2d su = M.ldlt().solve(rhs);
  return 0.5 * ((C1 + su(0) * d1) + (C2 + su(1) * d2));
}

bool cheiralityOK(const Eigen::Isometry3d& T1_cw, const Eigen::Isometry3d& T2_cw,
                  const Eigen::Vector3d& X_w) {
  return (T1_cw * X_w).z() > 0 && (T2_cw * X_w).z() > 0;
}

double parallaxAngle(const Eigen::Isometry3d& T1_cw, const Eigen::Isometry3d& T2_cw,
                     const Eigen::Vector3d& X_w) {
  const Eigen::Vector3d C1 = -T1_cw.linear().transpose() * T1_cw.translation();
  const Eigen::Vector3d C2 = -T2_cw.linear().transpose() * T2_cw.translation();
  const Eigen::Vector3d a = X_w - C1, b = X_w - C2;
  // atan2(|a x b|, a.b) is accurate for tiny angles (acos is not).
  return std::atan2(a.cross(b).norm(), a.dot(b));
}
