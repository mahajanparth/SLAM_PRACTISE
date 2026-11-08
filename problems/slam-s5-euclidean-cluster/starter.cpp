#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

class UnionFind {
 public:
  explicit UnionFind(int n) {
    // TODO
  }
  int find(int x) {
    // TODO
    return x;
  }
  bool unite(int a, int b) {
    // TODO
    return false;
  }
  int size(int x) {
    // TODO
    return 1;
  }
};

std::vector<int> euclideanCluster(const std::vector<Eigen::Vector3d>& pts, double r,
                                  int min_size = 1, int max_size = std::numeric_limits<int>::max()) {
  // TODO
  return {};
}
