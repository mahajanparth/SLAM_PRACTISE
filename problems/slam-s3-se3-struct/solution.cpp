#include <Eigen/Dense>
#include <Eigen/Geometry>

struct SE3 {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  // this(other(p)) = R (R2 p + t2) + t
  SE3 operator*(const SE3& other) const {
    SE3 out;
    out.R = R * other.R;
    out.t = R * other.t + t;
    return out;
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d& p) const { return R * p + t; }

  // p_b = R^T (p_a - t): rotation inverse is the transpose. O(1), no general matrix inversion.
  SE3 inverse() const {
    SE3 out;
    out.R = R.transpose();
    out.t = -(out.R * t);
    return out;
  }

  Eigen::Matrix4d matrix() const {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();  // bottom row = [0 0 0 1]
    M.topLeftCorner<3, 3>() = R;
    M.topRightCorner<3, 1>() = t;
    return M;
  }
};
