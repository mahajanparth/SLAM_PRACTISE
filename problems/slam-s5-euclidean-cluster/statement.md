# Euclidean Clustering with Union-Find

Object segmentation after ground removal: points belong to the same object if they are linked by a chain of neighbours closer than $r$. Implement it with **your own union-find**.

```cpp
class UnionFind {
 public:
  explicit UnionFind(int n);
  int find(int x);              // representative of x's set
  bool unite(int a, int b);     // merge; false if already in the same set
  int size(int x);              // size of x's set
};

std::vector<int> euclideanCluster(const std::vector<Eigen::Vector3d>& pts, double r,
                                  int min_size = 1, int max_size = std::numeric_limits<int>::max());
```

- Points $i, j$ are directly connected iff $\lVert p_i - p_j\rVert \le r$. Clusters are the connected components (transitive closure).
- Return one label per point. Clusters whose size is outside `[min_size, max_size]` get label **-1**.
- **Canonical labels:** the surviving clusters are numbered `0, 1, 2, …` in order of their **lowest point index**. So `pts[0]` gets label 0 unless its cluster was filtered out.
- Empty input gives an empty result. `r > 0`.

### Constraints
- Up to ~10⁵ points. Aim for about $O(N\cdot k)$ ($k$ = neighbours per point), not $O(N^2)$.
- Union-find ops should be near $O(1)$ amortised (path compression + union by rank/size).

### Think about
- What's the amortised complexity with both optimisations, and with only one?
- Voxel hash (27 cells) vs KD-tree radius search vs BFS flood fill: trade-offs?
- Why can a chain of points spaced $0.9r$ merge two "objects"? How does PCL mitigate under-segmentation?
- How would you parallelise this?
