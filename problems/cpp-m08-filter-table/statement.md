# Filter Dispatch Table

A preprocessing node reads its filter chain from a config file, e.g. `["max_range 40", "min_z -1.5", "max_z 2.0"]`. You need to turn **names** into **callables** at runtime and then run the chain. `struct Point3 { double x, y, z; };` is defined, along with:
```cpp
using UnaryOp        = double (*)(double);                     // raw function pointer
using PointPredicate = std::function<bool(const Point3&)>;     // type-erased callable: true = keep
```

| Function | Behaviour |
|---|---|
| `double applyN(UnaryOp f, double x, int n)` | Return `f(f(…f(x)))` with `f` applied `n` times. If `n ≤ 0` **or `f` is null**, return `x`. |
| `double sumField(const std::vector<Point3>& cloud, double Point3::*field)` | Sum of the chosen coordinate over the cloud, e.g. `sumField(c, &Point3::z)`. |
| `PointPredicate makeFilter(const std::string& name, double param)` | Factory from a name to a predicate (true = keep). It must support the table below. For an **unknown** name, return an **empty** `PointPredicate`. |
| `std::vector<Point3> applyPipeline(const std::vector<Point3>& cloud, const std::vector<PointPredicate>& stages)` | Keep the points for which **every non-empty** stage returns true, preserving order. Empty stages are skipped (never called). |

| name | keeps `p` iff |
|---|---|
| `"max_range"` | $\sqrt{x^2+y^2+z^2} \le$ `param` |
| `"min_z"` | `p.z >= param` |
| `"max_z"` | `p.z <= param` |
| `"min_x"` | `p.x >= param` (e.g. drop points behind the vehicle) |

### Constraints
- Cloud up to 10⁵ points. `applyPipeline` is O(n · stages).
- Names are case-sensitive.

### Think about
- Why does calling an empty `std::function` throw, while a null function pointer is UB?
- How would you make the factory **extensible** (registering new filters from other translation units)?
- What's the cost difference between running stages as `std::function` and as a template parameter pack?
- Could `UnaryOp` hold a capturing lambda? Why not?
