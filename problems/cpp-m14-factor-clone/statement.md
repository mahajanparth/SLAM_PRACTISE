# 1-D Factor Graph with Polymorphic Factors

A toy pose-graph back-end along a corridor: the state is `x` (one position per pose), and constraints are
**factors**. The abstract base class is given:

```cpp
class Factor {
 public:
  virtual ~Factor() = default;
  virtual double error(const std::vector<double>& x) const = 0;   // whitened residual r
  virtual std::unique_ptr<Factor> clone() const = 0;               // deep "virtual copy"
  virtual std::string type() const = 0;
};
```

Implement:

| Class / function | Behaviour |
|---|---|
| `PriorFactor(int key, double mean, double sigma)` | `r = (x[key] - mean) / sigma`, `type()` = `"prior"` |
| `BetweenFactor(int i, int j, double meas, double sigma)` | `r = (x[j] - x[i] - meas) / sigma`, `type()` = `"between"` |
| `clone()` in both | Returns a new heap copy of the **same dynamic type** |
| `FactorGraph(const FactorGraph&)`, `operator=(const FactorGraph&)` | **Deep** copy (cloned factors, no shared objects); self-assignment safe |
| `double FactorGraph::totalError(const std::vector<double>& x) const` | $\tfrac12\sum_k r_k^2$ (0 for an empty graph) |
| `std::map<std::string,int> FactorGraph::countByType() const` | e.g. `{{"between",2},{"prior",1}}` |
| `std::unique_ptr<Factor> parseFactor(const std::string& line)` | Factory, see below |

`add`, `size`, `at` and the defaulted moves are already written.

**Factory format** (whitespace-separated):
```
prior   <key> <mean> <sigma>
between <i> <j> <meas> <sigma>
```
Return `nullptr` for: unknown type, missing or non-numeric fields, trailing extra tokens, `sigma <= 0`,
or negative keys.

The tests also put their own `Factor` subclass into your graph and copy it.

### Assumptions
- Keys used in `error` are valid indices into `x`.

### Think about
- Why can't the graph copy a factor with `std::make_unique<Factor>(*f)`?
- Why does declaring a copy constructor stop the compiler from generating moves, and how is that handled here?
- What are the exception-safety guarantees of your copy assignment?
- GTSAM uses `shared_ptr` factors and a shallow-copying graph. Trade-offs?
