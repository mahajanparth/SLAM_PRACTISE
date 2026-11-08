#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <functional>

// Provided: Rodrigues formula, Exp: so(3) -> SO(3).
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  const double th = w.norm();
  if (th < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(th, w / th).toRotationMatrix();
}

Eigen::MatrixXd numericalJacobian(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
                                  const Eigen::VectorXd& x, double eps = 1e-6) {
  const Eigen::Index n = x.size();
  const Eigen::Index m = f(x).size();  // one evaluation to learn the output dimension
  Eigen::MatrixXd J(m, n);
  Eigen::VectorXd xp = x, xm = x;
  for (Eigen::Index k = 0; k < n; ++k) {
    xp[k] = x[k] + eps;
    xm[k] = x[k] - eps;
    J.col(k) = (f(xp) - f(xm)) / (2.0 * eps);  // truncation error O(eps^2)
    xp[k] = xm[k] = x[k];
  }
  return J;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(),
       v.z(), 0, -v.x(),
       -v.y(), v.x(), 0;
  return S;
}

// R Exp(d) p ~= R (I + [d]x) p = R p + R (d x p) = R p - R [p]x d
Eigen::Matrix3d rotatePointJacobianAnalytic(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
  return -R * skew(p);
}

Eigen::Matrix3d rotatePointJacobianNumerical(const Eigen::Matrix3d& R, const Eigen::Vector3d& p,
                                             double eps = 1e-6) {
  // Perturb on the manifold through Exp, never the raw matrix entries.
  auto f = [&](const Eigen::VectorXd& d) -> Eigen::VectorXd {
    return R * expSO3(Eigen::Vector3d(d)) * p;
  };
  return numericalJacobian(f, Eigen::VectorXd::Zero(3), eps);
}
