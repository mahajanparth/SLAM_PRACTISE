# Cloud Lambdas

A perception node preprocesses each lidar scan with small one-off predicates and comparators, which are a natural fit for lambdas. `struct Point3 { double x, y, z; };` is already defined. Implement:

| Function | Behaviour |
|---|---|
| `void sortByDistance(std::vector<Point3>& cloud, const Point3& origin)` | Sort ascending by Euclidean distance to `origin`. Points at **equal** distance keep their original relative order. |
| `std::size_t removeOutOfRange(std::vector<Point3>& cloud, double min_r, double max_r)` | Erase every point whose range from `(0,0,0)` is **not** in `[min_r, max_r]`. Keep the order of the survivors. Return how many were removed. |
| `auto makeBoxFilter(Point3 lo, Point3 hi)` | Return a callable `bool(const Point3&)` that is true iff `lo.x ≤ p.x ≤ hi.x` (and the same for y and z). |
| `auto makeIdGenerator(int first)` | Return a callable `int()` that yields `first, first+1, first+2, …`. Generators are independent, and a **copy** of a generator continues from the state it was copied at. |

### Constraints
- Up to 10⁵ points. Sorting is O(n log n), and everything else is O(n).
- No raw loops are needed: use `<algorithm>` plus lambdas.
- The callables you return must stay valid after the arguments you were given go out of scope.

### Think about
- `[=]` vs `[&]` vs explicit captures: which one is safe for the callables you **return**?
- Why can `std::sort` with a lambda comparator be faster than `qsort` with a function pointer?
- What does `mutable` actually change in the generated closure class?
- Why compare squared distances in the comparator?
