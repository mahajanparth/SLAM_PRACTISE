#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

class KDTree {
 public:
  explicit KDTree(const std::vector<Eigen::Vector3d>& pts) : pts_(pts) {
    // TODO: build the tree
  }

  int nearest(const Eigen::Vector3d& q) const {
    // TODO
    return -1;
  }

  std::vector<int> radiusSearch(const Eigen::Vector3d& q, double r) const {
    // TODO
    return {};
  }

 private:
  std::vector<Eigen::Vector3d> pts_;
  // TODO: your node storage
};
