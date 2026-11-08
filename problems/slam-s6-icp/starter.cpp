#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <vector>

using Cloud = std::vector<Eigen::Vector3d>;

Eigen::Isometry3d icp(const Cloud& src, const Cloud& tgt, const Eigen::Isometry3d& init,
                      int max_iters,
                      double max_corr_dist = std::numeric_limits<double>::infinity()) {
  // TODO
  return init;
}
