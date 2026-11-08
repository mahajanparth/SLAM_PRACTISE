# Brute-Force k Nearest Neighbours

Before you build a KD-tree you need a correct reference, and for small clouds brute force is often the fastest option anyway.

```cpp
// Indices of the k points nearest to q, sorted by increasing distance.
std::vector<int> knn(const std::vector<Eigen::Vector3d>& pts, const Eigen::Vector3d& q, int k);
```

### Semantics
- Sorted by Euclidean distance, ascending. **Ties** (equal distance) are broken by **smaller index first**.
- If `k > N`, return all N indices (sorted). If `k <= 0` or the cloud is empty, return `{}`.
- Complexity target: O(N log k) time (not O(N log N)), O(N) or O(k) extra memory.

### Think about
- `partial_sort` vs `nth_element` + sort vs a size-k max-heap: complexity and constant factors.
- When does brute force beat a KD-tree (small N, a single query, GPU)?
- How would you batch many queries efficiently (cache blocking, `Matrix3Xf` with GEMM for distances, $\lVert a-b\rVert^2 = \lVert a\rVert^2 + \lVert b\rVert^2 - 2a^Tb$)?
- Radius search variant: what changes in the output contract?
