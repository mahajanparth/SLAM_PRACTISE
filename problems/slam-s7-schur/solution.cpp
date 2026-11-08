#include <Eigen/Dense>

struct Reduced {
  Eigen::MatrixXd H;
  Eigen::VectorXd b;
};

Reduced marginalize(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, int na) {
  const Eigen::Index n = H.rows(), nc = n - na;
  if (nc == 0) return {H, b};
  if (na == 0) return {Eigen::MatrixXd(0, 0), Eigen::VectorXd(0)};  // avoid 0-column solves
  const auto A = H.topLeftCorner(na, na);
  const auto B = H.topRightCorner(na, nc);
  const auto C = H.bottomRightCorner(nc, nc);
  // Factor C once; C^{-1} B^T and C^{-1} b_c via solves (never form C^{-1}).
  const Eigen::LDLT<Eigen::MatrixXd> Cf(C);
  const Eigen::MatrixXd CinvBt = Cf.solve(B.transpose());
  const Eigen::VectorXd Cinvbc = Cf.solve(b.tail(nc));
  Reduced r;
  r.H = A - B * CinvBt;
  r.b = b.head(na) - B * Cinvbc;
  return r;
}

Eigen::VectorXd solveViaSchur(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, int na) {
  const Eigen::Index n = H.rows(), nc = n - na;
  if (na == 0 || nc == 0) return H.ldlt().solve(b);  // nothing to split
  const Reduced r = marginalize(H, b, na);
  Eigen::VectorXd x(n);
  x.head(na) = r.H.ldlt().solve(r.b);
  {
    // Back-substitution: C dc = b_c - B^T da
    const Eigen::VectorXd rhs = b.tail(nc) - H.topRightCorner(na, nc).transpose() * x.head(na);
    x.tail(nc) = H.bottomRightCorner(nc, nc).ldlt().solve(rhs);
  }
  return x;
}
