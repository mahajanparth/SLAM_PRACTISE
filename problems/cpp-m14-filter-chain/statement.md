# Range Filter Chain

A lidar driver passes each scan's range readings (`std::vector<double>`, metres) through a configurable
chain of filters. Each filter is a class derived from the abstract interface below, which is already
defined in the starter:

```cpp
class Filter {
 public:
  virtual ~Filter() = default;
  virtual std::vector<double> apply(const std::vector<double>& in) const = 0;
  virtual std::string name() const = 0;
};
```

Implement:

| Class | `apply(in)` | `name()` |
|---|---|---|
| `ClampFilter(double lo, double hi)` | each value clamped to `[lo, hi]` | `"clamp"` |
| `ScaleFilter(double s)` | each value multiplied by `s` | `"scale"` |
| `MovingAverage(std::size_t window)` | **causal** mean: `out[i]` = mean of `in[max(0, i-window+1) .. i]` | `"movavg"` |
| `FilterChain` | see below | – |

`FilterChain` **owns** its filters:

| Member | Behaviour |
|---|---|
| `void add(std::unique_ptr<Filter> f)` | Append `f` to the end of the chain (ignore `nullptr`) |
| `std::vector<double> apply(const std::vector<double>& in) const` | Run the filters in insertion order; an empty chain returns `in` unchanged |
| `std::string describe() const` | Filter names joined with `"->"`, e.g. `"clamp->scale"`; `""` if empty |
| `std::size_t size() const` | Number of filters |

The tests also add their own `Filter` subclasses to your chain, so it must work for **any** derived class.

### Constraints
- `window ≥ 1`. Scans contain up to 10⁵ readings; `MovingAverage::apply` must be O(n), not O(n·window).
- Empty input → empty output for every filter.

### Think about
- Why must `~Filter` be virtual here? What would ASan report without it?
- Why `std::vector<std::unique_ptr<Filter>>` and not `std::vector<Filter>`?
- The virtual call happens once per *scan*, not once per *point*. Why does that matter?
- How would you make `FilterChain` copyable?
