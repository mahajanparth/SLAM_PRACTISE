# KD-Tree: build, nearest, radius search

Every ICP iteration, normal estimation and clustering step in a lidar pipeline needs fast neighbour queries. Brute force is $O(N)$ per query; build a **static 3D KD-tree** instead.

```cpp
class KDTree {
 public:
  explicit KDTree(const std::vector<Eigen::Vector3d>& pts);              // build
  int nearest(const Eigen::Vector3d& q) const;                           // index into pts
  std::vector<int> radiusSearch(const Eigen::Vector3d& q, double r) const;
};
```

| Method | Returns |
|---|---|
| constructor | builds the tree over a copy of `pts` (indices refer to the original order) |
| `nearest(q)` | index of a point with minimal $\lVert p_i - q\rVert$; **-1** if the tree is empty. Any index is fine on ties. |
| `radiusSearch(q, r)` | all indices $i$ with $\lVert p_i - q\rVert \le r$, **any order**; empty if none or the tree is empty |

### Constraints
- Up to ~10⁵ points, any coordinates; duplicates allowed. `r >= 0` (`r = 0` returns exact duplicates of `q`).
- Build $O(N\log N)$, query $O(\log N)$ on average. Tests compare against brute force on random queries.
- You may change the private section freely.

### Think about
- What's the worst-case query cost, and when does it happen?
- Leaf buckets of 10–20 points vs one point per leaf: why are buckets faster in practice?
- Flat node array vs `unique_ptr` children. Which is more cache-friendly?
- How would you support incremental inserts (ikd-tree)? Why do KD-trees degrade in high dimensions?
- How does nanoflann/FLANN do approximate NN?
