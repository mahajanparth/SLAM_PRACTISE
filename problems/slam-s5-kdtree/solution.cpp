#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

class KDTree {
 public:
  explicit KDTree(const std::vector<Eigen::Vector3d>& pts) : pts_(pts), idx_(pts.size()) {
    std::iota(idx_.begin(), idx_.end(), 0);
    if (!pts_.empty()) {
      nodes_.reserve(2 * pts_.size() / kLeaf + 1);
      build(0, static_cast<int>(idx_.size()));
    }
  }

  int nearest(const Eigen::Vector3d& q) const {
    if (nodes_.empty()) return -1;
    int best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    nearestRec(0, q, best, best_d2);
    return best;
  }

  std::vector<int> radiusSearch(const Eigen::Vector3d& q, double r) const {
    std::vector<int> out;
    if (!nodes_.empty() && r >= 0) radiusRec(0, q, r * r, out);
    return out;
  }

 private:
  // Leaf: axis == -1 and [begin, end) indexes idx_. Inner: split plane + child node ids.
  struct Node {
    int axis = -1;
    double split = 0.0;
    int left = -1, right = -1;
    int begin = 0, end = 0;
  };
  static constexpr int kLeaf = 8;

  std::vector<Eigen::Vector3d> pts_;
  std::vector<int> idx_;     // permuted point indices; points themselves never move
  std::vector<Node> nodes_;  // flat storage, root = 0

  int build(int b, int e) {
    const int id = static_cast<int>(nodes_.size());
    nodes_.emplace_back();
    if (e - b <= kLeaf) {
      nodes_[id].begin = b;
      nodes_[id].end = e;
      return id;
    }
    // Split on the axis of largest spread (robust to flat/elongated clouds).
    Eigen::Vector3d lo = pts_[idx_[b]], hi = lo;
    for (int k = b + 1; k < e; ++k) {
      lo = lo.cwiseMin(pts_[idx_[k]]);
      hi = hi.cwiseMax(pts_[idx_[k]]);
    }
    int axis;
    (hi - lo).maxCoeff(&axis);
    const int m = b + (e - b) / 2;
    std::nth_element(idx_.begin() + b, idx_.begin() + m, idx_.begin() + e,
                     [&](int i, int j) { return pts_[i][axis] < pts_[j][axis]; });
    // Invariant: left range coords <= split <= right range coords. Ranges halve even with duplicates.
    const double split = pts_[idx_[m]][axis];
    const int l = build(b, m);
    const int r = build(m, e);
    // Note: no reference into nodes_ is held across the recursive calls (emplace_back may reallocate).
    nodes_[id].axis = axis;
    nodes_[id].split = split;
    nodes_[id].left = l;
    nodes_[id].right = r;
    return id;
  }

  void nearestRec(int ni, const Eigen::Vector3d& q, int& best, double& best_d2) const {
    const Node& n = nodes_[ni];
    if (n.axis < 0) {
      for (int k = n.begin; k < n.end; ++k) {
        const double d2 = (pts_[idx_[k]] - q).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best = idx_[k]; }
      }
      return;
    }
    const double diff = q[n.axis] - n.split;
    const int near_child = diff < 0 ? n.left : n.right;
    const int far_child = diff < 0 ? n.right : n.left;
    nearestRec(near_child, q, best, best_d2);
    // Every point on the far side is at least |diff| away along this axis.
    if (diff * diff < best_d2) nearestRec(far_child, q, best, best_d2);
  }

  void radiusRec(int ni, const Eigen::Vector3d& q, double r2, std::vector<int>& out) const {
    const Node& n = nodes_[ni];
    if (n.axis < 0) {
      for (int k = n.begin; k < n.end; ++k)
        if ((pts_[idx_[k]] - q).squaredNorm() <= r2) out.push_back(idx_[k]);
      return;
    }
    const double diff = q[n.axis] - n.split;
    if (diff <= 0 || diff * diff <= r2) radiusRec(n.left, q, r2, out);
    if (diff >= 0 || diff * diff <= r2) radiusRec(n.right, q, r2, out);
  }
};
