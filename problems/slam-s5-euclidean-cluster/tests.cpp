// The candidate's code is injected before this file (g++ -include).
#include <numeric>
#include <limits>
#include <queue>
#include <vector>
#include "common/check.h"
#include "common/cloud_utils.h"

// O(N^2) BFS reference with the same canonical labelling.
static std::vector<int> bruteCluster(const cu::Cloud& c, double r, int mn = 1, int mx = std::numeric_limits<int>::max()) {
  const int n = static_cast<int>(c.size());
  std::vector<int> comp(n, -1), lab(n, -1);
  int nc = 0;
  std::vector<int> sizes;
  for (int s = 0; s < n; ++s) {
    if (comp[s] >= 0) continue;
    std::queue<int> q;
    q.push(s); comp[s] = nc; int sz = 0;
    while (!q.empty()) {
      int u = q.front(); q.pop(); ++sz;
      for (int v = 0; v < n; ++v)
        if (comp[v] < 0 && (c[u] - c[v]).squaredNorm() <= r * r) { comp[v] = nc; q.push(v); }
    }
    sizes.push_back(sz); ++nc;
  }
  std::vector<int> remap(nc, -1);
  int next = 0;
  for (int i = 0; i < n; ++i) {
    const int k = comp[i];
    if (sizes[k] < mn || sizes[k] > mx) continue;
    if (remap[k] < 0) remap[k] = next++;
    lab[i] = remap[k];
  }
  return lab;
}

int main() {
  // Union-find basics.
  UnionFind uf(6);
  CHECK(uf.unite(0, 1));
  CHECK(uf.unite(2, 3));
  CHECK(uf.unite(1, 3));
  CHECK(!uf.unite(0, 2));          // already together
  CHECK(uf.find(0) == uf.find(3));
  CHECK(uf.find(4) != uf.find(0));
  CHECK_EQ(uf.size(2), 4);

  // Three well-separated blobs, interleaved order.
  cu::Cloud c;
  const Eigen::Vector3d centers[3] = {{0, 0, 0}, {5, 0, 0}, {0, -5, 2}};
  cu::Cloud blob = cu::uniformCube(90, 0.5);
  for (int i = 0; i < 90; ++i) c.push_back(centers[(i * 7) % 3] + blob[i]);
  std::vector<int> lab = euclideanCluster(c, 0.6);
  CHECK(lab == bruteCluster(c, 0.6));

#ifdef HIDDEN
  // Chain spaced 0.9r -> one cluster; spaced 1.1r -> all singletons.
  {
    cu::Cloud chain, sparse;
    for (int k = 0; k < 40; ++k) { chain.push_back({0.9 * k, -3.0, 1.0}); sparse.push_back({-1.1 * k, 0.0, 0.0}); }
    CHECK(euclideanCluster(chain, 1.0) == std::vector<int>(40, 0));
    std::vector<int> expect(40);
    std::iota(expect.begin(), expect.end(), 0);
    CHECK(euclideanCluster(sparse, 1.0) == expect);
  }
  // Empty and single point.
  CHECK(euclideanCluster({}, 1.0).empty());
  CHECK(euclideanCluster({{-7, -7, -7}}, 0.5) == std::vector<int>({0}));
  // Duplicate points / distance exactly r (inclusive), across negative voxel boundaries.
  CHECK(euclideanCluster({{-0.25, 0, 0}, {0.25, 0, 0}, {-0.25, 0, 0}}, 0.5) == std::vector<int>({0, 0, 0}));
  // Min/max size filter: first cluster filtered out -> labels start at the next one.
  {
    cu::Cloud f = {{100, 100, 100}, {0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {50, 0, 0}, {50.1, 0, 0}};
    CHECK(euclideanCluster(f, 0.15, 2) == std::vector<int>({-1, 0, 0, 0, 1, 1}));
    CHECK(euclideanCluster(f, 0.15, 2, 2) == std::vector<int>({-1, -1, -1, -1, 0, 0}));
  }
  // Larger random cloud, negative coordinates, vs brute force.
  {
    cu::Cloud big = cu::uniformCube(1000, 2.5);
    CHECK(euclideanCluster(big, 0.25) == bruteCluster(big, 0.25));
    CHECK(euclideanCluster(big, 0.4, 5, 200) == bruteCluster(big, 0.4, 5, 200));
  }
  // Long union chain (path compression must not overflow the stack / be quadratic).
  {
    UnionFind u(100000);
    for (int i = 1; i < 100000; ++i) u.unite(i - 1, i);
    CHECK_EQ(u.size(0), 100000);
    CHECK(u.find(0) == u.find(99999));
  }
#endif
  return TEST_SUMMARY();
}
