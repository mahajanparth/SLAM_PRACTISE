# Generic Lambdas: a Scan-Statistics Toolkit

You're writing a small header-only toolkit for summarising lidar scans per laser ring. A point is:
```cpp
struct LidarPoint { double x, y, z; float intensity; int ring; };
```
Implement these four templates (the signatures are in the starter):

| Function | Behaviour |
|---|---|
| `compose(f, g)` | Returns a callable `h` such that `h(x) == g(f(x))` for **any** argument type `f` accepts. |
| `allOf(preds...)` | Returns a callable `p` such that `p(x)` is true iff **every** predicate returns true for `x`. With zero predicates it always returns `true`. Evaluation short-circuits left to right. |
| `groupReduce(items, key, init, op)` | Returns `std::map<K, Acc>` where `K = decay of key(item)`. For each item, in input order, `acc[key(item)] = op(acc[key(item)], item)`, and each accumulator starts at `init`. Keys never seen are absent. |
| `makeRunningMean()` | Returns a callable `double(double)` that returns the mean of **all** values passed to it so far (including this one). Each call to `makeRunningMean()` produces an independent accumulator. |

Example:
```cpp
auto range = [](const LidarPoint& p) { return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z); };
auto isFar = compose(range, [](double r) { return r > 30.0; });
auto counts = groupReduce(scan, [](const auto& p) { return p.ring; }, 0,
                          [](int c, const LidarPoint&) { return c + 1; });   // std::map<int,int>
```

### Constraints
- The callables must **own** what they use (they're returned from the factory and outlive its arguments).
- `groupReduce` is O(n log k) for k distinct keys.
- `makeRunningMean` is O(1) memory, so it must not store the samples.

### Think about
- What is the closure type of a generic lambda, and when is its `operator()` instantiated?
- Why does `allOf` with no arguments return `true`? (Hint: fold-expression identity elements.)
- A naive running mean `sum / n` loses precision after 10⁸ samples. Why does the incremental form behave better?
- How would you write `compose` **without** lambdas (C++03)? Compare the boilerplate.
